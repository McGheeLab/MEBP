"""
PrintPlanOfAction.py -- Comprehensive print execution plan generator for MEBP v7.2.9.

The Plan of Action is the single source of truth for how a print job executes.
It encodes EVERY aspect of the print workflow:

    - Ink swap strategy (moved from HardwareConfig in v7.2.9)
    - Ink gathering: per-ink max pickup volume + extra percentage
    - Z travel behavior: safe Z, top Z, wait-for-confirm, fast travel in/out
    - XY travel behavior: fast travel between wells
    - Plunge-in behavior: buffer distance before print Z
    - Service sequence: waste/wash/buffer/ink_load toggles + volumes
    - Final cleanup sequence at end of print
    - Run splitting based on syringe capacity

v7.2.9 Changes:
    - NEW PrintExecutionConfig replaces PlanPreferences
    - NEW InkGatherConfig for per-ink-type pickup amounts + extra %
    - NEW ZTravelConfig for Z-axis behavior (safe Z, fast travel, plunge buffer)
    - NEW XYTravelConfig for XY travel behavior
    - InkSwapStrategy now lives here (moved from HardwareConfig)
    - NEW step types: GATHER_INK, FINAL_CLEANUP, TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP
    - Richer PlanStep with z_behavior, travel_mode, wait_for_confirm fields
    - Comprehensive _compute() uses all config to generate detailed plan
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
#  Step Types
# ═══════════════════════════════════════════════════════════════════

class PlanStepType(Enum):
    """Types of steps in a print execution plan."""
    # Original step types (backwards compatible)
    LOAD_INK = "load_ink"
    WASH = "wash"
    WASTE = "waste"
    REFILL_BUFFER = "refill_buffer"
    PRINT = "print"
    RETURN_HOME = "return_home"
    # v7.2.9: New granular step types
    GATHER_INK = "gather_ink"           # Ink pickup with per-ink specs + extra %
    FINAL_CLEANUP = "final_cleanup"     # End-of-print cleanup sequence
    TRAVEL_XY = "travel_xy"            # Explicit fast XY move between wells
    MOVE_SAFE_Z = "move_safe_z"        # Raise to safe Z for travel
    INK_SWAP = "ink_swap"              # Full ink swap sequence (waste→wash→buffer→wash→load)


PLAN_STEP_COLORS = {
    PlanStepType.LOAD_INK: "#89b4fa",
    PlanStepType.WASH: "#f9e2af",
    PlanStepType.WASTE: "#f38ba8",
    PlanStepType.REFILL_BUFFER: "#cba6f7",
    PlanStepType.PRINT: "#a6e3a1",
    PlanStepType.RETURN_HOME: "#6c7086",
    PlanStepType.GATHER_INK: "#74c7ec",
    PlanStepType.FINAL_CLEANUP: "#fab387",
    PlanStepType.TRAVEL_XY: "#94e2d5",
    PlanStepType.MOVE_SAFE_Z: "#b4befe",
    PlanStepType.INK_SWAP: "#f5c2e7",
}

PLAN_STEP_ICONS = {
    PlanStepType.LOAD_INK: "\U0001f535",       # blue circle
    PlanStepType.WASH: "\U0001f7e1",            # yellow circle
    PlanStepType.WASTE: "\U0001f534",           # red circle
    PlanStepType.REFILL_BUFFER: "\U0001f7e3",   # purple circle
    PlanStepType.PRINT: "\U0001f7e2",           # green circle
    PlanStepType.RETURN_HOME: "\u2b1c",         # white square
    PlanStepType.GATHER_INK: "\U0001f4a7",      # droplet
    PlanStepType.FINAL_CLEANUP: "\U0001f9f9",   # broom
    PlanStepType.TRAVEL_XY: "\u27a1\ufe0f",     # right arrow
    PlanStepType.MOVE_SAFE_Z: "\u2b06\ufe0f",   # up arrow
    PlanStepType.INK_SWAP: "\U0001f504",        # counterclockwise arrows
}


# ═══════════════════════════════════════════════════════════════════
#  Ink Swap Strategy (moved from HardwareConfig.py in v7.2.9)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class InkSwapStrategy:
    """
    Configurable ink swap sequence for single-syringe multi-ink workflows.

    When a pump needs to switch between inks, the planner inserts a
    cleaning/loading sequence. Each step can be toggled on/off.

    Full sequence: waste -> wash -> buffer -> wash -> ink_load -> wash -> print
    """
    waste: bool = True          # Dispense remaining ink to waste reservoir
    wash_pre: bool = True       # Wash line before buffer
    buffer: bool = True         # Flush with buffer solution
    wash_post: bool = True      # Wash line after buffer
    ink_load: bool = True       # Load new ink into syringe (always recommended)
    wash_final: bool = True     # Final wash before resuming print

    # Volumes for each step (uL) -- sensible defaults
    waste_volume_uL: float = 50.0
    wash_volume_uL: float = 100.0
    buffer_volume_uL: float = 100.0
    ink_load_volume_uL: float = 50.0

    def get_enabled_steps(self) -> list[str]:
        """Return ordered list of enabled step names."""
        steps = []
        if self.waste:      steps.append("waste")
        if self.wash_pre:   steps.append("wash")
        if self.buffer:     steps.append("buffer")
        if self.wash_post:  steps.append("wash")
        if self.ink_load:   steps.append("ink_load")
        if self.wash_final: steps.append("wash")
        return steps

    def to_dict(self) -> dict:
        return {
            "waste": self.waste,
            "wash_pre": self.wash_pre,
            "buffer": self.buffer,
            "wash_post": self.wash_post,
            "ink_load": self.ink_load,
            "wash_final": self.wash_final,
            "waste_volume_uL": self.waste_volume_uL,
            "wash_volume_uL": self.wash_volume_uL,
            "buffer_volume_uL": self.buffer_volume_uL,
            "ink_load_volume_uL": self.ink_load_volume_uL,
        }

    @classmethod
    def from_dict(cls, data: dict) -> InkSwapStrategy:
        return cls(
            waste=data.get("waste", True),
            wash_pre=data.get("wash_pre", True),
            buffer=data.get("buffer", True),
            wash_post=data.get("wash_post", True),
            ink_load=data.get("ink_load", True),
            wash_final=data.get("wash_final", True),
            waste_volume_uL=data.get("waste_volume_uL", 50.0),
            wash_volume_uL=data.get("wash_volume_uL", 100.0),
            buffer_volume_uL=data.get("buffer_volume_uL", 100.0),
            ink_load_volume_uL=data.get("ink_load_volume_uL", 50.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  v7.2.9: Per-Ink Gather Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class InkGatherConfig:
    """Per-ink-type configuration for ink gathering.

    Controls how much ink is picked up and whether extra is added.
    Each ink type can have its own settings.
    """
    ink_name: str = ""
    max_pickup_uL: float = 0.0     # 0 = use syringe capacity
    extra_percent: float = 10.0     # Extra ink % (e.g. 10 = pick up 10% more)

    def effective_volume(self, needed_uL: float, syringe_cap_uL: float) -> float:
        """Compute actual pickup volume: needed * (1 + extra%) clamped to max."""
        vol = needed_uL * (1.0 + self.extra_percent / 100.0)
        cap = self.max_pickup_uL if self.max_pickup_uL > 0 else syringe_cap_uL
        return min(vol, cap)

    def to_dict(self) -> dict:
        return {
            "ink_name": self.ink_name,
            "max_pickup_uL": self.max_pickup_uL,
            "extra_percent": self.extra_percent,
        }

    @classmethod
    def from_dict(cls, data: dict) -> InkGatherConfig:
        return cls(
            ink_name=data.get("ink_name", ""),
            max_pickup_uL=data.get("max_pickup_uL", 0.0),
            extra_percent=data.get("extra_percent", 10.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  v7.2.9: Z Travel Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class ZTravelConfig:
    """Configuration for Z-axis travel behavior during printing.

    Controls how the needle moves vertically between wells, into wells,
    and whether to wait for position confirmation.
    """
    # Safe Z height for inter-well travel (mm above well plate)
    safe_z_mm: float = 5.0

    # Top-of-well Z height (mm) -- used for 2-phase Z approach.
    # 0 = not calibrated, use single-phase movement.
    top_z_mm: float = 0.0

    # Whether to wait for Z to reach target and confirm position before
    # proceeding to the next move. Slower but safer.
    wait_for_z_confirm: bool = True

    # Whether the needle can fast-travel in Z OUT of a well
    # (True = fast exit, False = slow controlled exit)
    fast_z_exit_well: bool = True

    # Whether the needle can fast-travel in Z INTO a well
    # (True = fast entry to top_z then slow, False = slow all the way)
    fast_z_enter_well: bool = False

    # Plunge-in behavior: use a safe buffer Z distance before reaching
    # the print Z height. The needle travels fast to (print_z + buffer),
    # then slowly covers the last buffer distance.
    use_plunge_buffer: bool = True

    # Buffer distance in microns above print Z for slow plunge-in
    plunge_buffer_um: float = 500.0  # 500um = 0.5mm default

    @property
    def plunge_buffer_mm(self) -> float:
        """Buffer distance in mm."""
        return self.plunge_buffer_um / 1000.0

    def to_dict(self) -> dict:
        return {
            "safe_z_mm": self.safe_z_mm,
            "top_z_mm": self.top_z_mm,
            "wait_for_z_confirm": self.wait_for_z_confirm,
            "fast_z_exit_well": self.fast_z_exit_well,
            "fast_z_enter_well": self.fast_z_enter_well,
            "use_plunge_buffer": self.use_plunge_buffer,
            "plunge_buffer_um": self.plunge_buffer_um,
        }

    @classmethod
    def from_dict(cls, data: dict) -> ZTravelConfig:
        return cls(
            safe_z_mm=data.get("safe_z_mm", 5.0),
            top_z_mm=data.get("top_z_mm", 0.0),
            wait_for_z_confirm=data.get("wait_for_z_confirm", True),
            fast_z_exit_well=data.get("fast_z_exit_well", True),
            fast_z_enter_well=data.get("fast_z_enter_well", False),
            use_plunge_buffer=data.get("use_plunge_buffer", True),
            plunge_buffer_um=data.get("plunge_buffer_um", 500.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  v7.2.9: XY Travel Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class XYTravelConfig:
    """Configuration for XY travel between wells."""
    # Use fast XY travel between wells (max stage speed)
    fast_xy_travel: bool = True

    # Fast XY travel speed (mm/s). Only used when fast_xy_travel is True.
    fast_xy_speed_mm_s: float = 10.0

    def to_dict(self) -> dict:
        return {
            "fast_xy_travel": self.fast_xy_travel,
            "fast_xy_speed_mm_s": self.fast_xy_speed_mm_s,
        }

    @classmethod
    def from_dict(cls, data: dict) -> XYTravelConfig:
        return cls(
            fast_xy_travel=data.get("fast_xy_travel", True),
            fast_xy_speed_mm_s=data.get("fast_xy_speed_mm_s", 10.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  v7.2.9: Final Cleanup Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class FinalCleanupConfig:
    """Configuration for the end-of-print cleanup sequence."""
    enabled: bool = True
    do_waste: bool = True       # Dispense remaining ink to waste
    do_wash: bool = True        # Wash needle after final waste
    do_dry_run: bool = False    # Run pump forward to clear residual
    wash_cycles: int = 3
    wash_time_per_cycle_s: float = 5.0

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "do_waste": self.do_waste,
            "do_wash": self.do_wash,
            "do_dry_run": self.do_dry_run,
            "wash_cycles": self.wash_cycles,
            "wash_time_per_cycle_s": self.wash_time_per_cycle_s,
        }

    @classmethod
    def from_dict(cls, data: dict) -> FinalCleanupConfig:
        return cls(
            enabled=data.get("enabled", True),
            do_waste=data.get("do_waste", True),
            do_wash=data.get("do_wash", True),
            do_dry_run=data.get("do_dry_run", False),
            wash_cycles=data.get("wash_cycles", 3),
            wash_time_per_cycle_s=data.get("wash_time_per_cycle_s", 5.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  v7.2.9: Comprehensive Execution Configuration
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PrintExecutionConfig:
    """
    Comprehensive configuration for print execution -- the single source
    of truth for all plan generation behavior.

    Replaces the old PlanPreferences with a much richer set of controls.
    Includes ink swap strategy, Z behavior, XY travel, ink gathering,
    final cleanup, and per-ink overrides.
    """
    # ── Ink Swap Strategy ──────────────────────────────────────────
    ink_swap: InkSwapStrategy = field(default_factory=InkSwapStrategy)

    # ── Z Travel Behavior ─────────────────────────────────────────
    z_travel: ZTravelConfig = field(default_factory=ZTravelConfig)

    # ── XY Travel Behavior ────────────────────────────────────────
    xy_travel: XYTravelConfig = field(default_factory=XYTravelConfig)

    # ── Final Cleanup ─────────────────────────────────────────────
    final_cleanup: FinalCleanupConfig = field(default_factory=FinalCleanupConfig)

    # ── Per-Ink Gathering ─────────────────────────────────────────
    # Key = ink name, value = gather config for that ink.
    # Inks not in this dict use the default_gather_config.
    ink_gather_configs: dict[str, InkGatherConfig] = field(default_factory=dict)
    default_gather_config: InkGatherConfig = field(
        default_factory=lambda: InkGatherConfig(
            ink_name="default", max_pickup_uL=0.0, extra_percent=10.0))

    # ── Service Sequence ──────────────────────────────────────────
    # Canonical source is ink_swap (.waste / .wash_pre / .wash_post /
    # .buffer / .ink_load / .wash_final). The legacy use_waste /
    # use_wash / use_buffer flags are surfaced as @property shims so
    # older callers continue to work; they read/write through ink_swap.
    wash_cycles: int = 3

    # ── Speeds ────────────────────────────────────────────────────
    travel_speed_mm_s: float = 10.0
    aspirate_speed_uL_s: float = 1.0
    dispense_speed_uL_s: float = 2.0
    wash_time_per_cycle_s: float = 5.0

    # ── Per-Pump Volume Overrides ─────────────────────────────────
    # Empty dict means "use HardwareConfig syringe capacities". Renamed
    # from max_ink_volume_uL in v7.5.0; the old name still works via an
    # alias property below.
    pump_volume_overrides_uL: dict[str, float] = field(default_factory=dict)

    # ── Legacy Aliases (deprecated, removed in v7.6.0) ────────────
    @property
    def max_ink_volume_uL(self) -> dict[str, float]:
        return self.pump_volume_overrides_uL

    @max_ink_volume_uL.setter
    def max_ink_volume_uL(self, value: dict[str, float]) -> None:
        self.pump_volume_overrides_uL = value

    @property
    def use_waste(self) -> bool:
        return self.ink_swap.waste

    @use_waste.setter
    def use_waste(self, value: bool) -> None:
        self.ink_swap.waste = bool(value)

    @property
    def use_wash(self) -> bool:
        return self.ink_swap.wash_pre or self.ink_swap.wash_post

    @use_wash.setter
    def use_wash(self, value: bool) -> None:
        self.ink_swap.wash_pre = bool(value)
        self.ink_swap.wash_post = bool(value)

    @property
    def use_buffer(self) -> bool:
        return self.ink_swap.buffer

    @use_buffer.setter
    def use_buffer(self, value: bool) -> None:
        self.ink_swap.buffer = bool(value)

    def get_ink_gather(self, ink_name: str) -> InkGatherConfig:
        """Get the gather config for a specific ink, falling back to default."""
        return self.ink_gather_configs.get(ink_name, self.default_gather_config)

    def to_dict(self) -> dict:
        return {
            "ink_swap": self.ink_swap.to_dict(),
            "z_travel": self.z_travel.to_dict(),
            "xy_travel": self.xy_travel.to_dict(),
            "final_cleanup": self.final_cleanup.to_dict(),
            "ink_gather_configs": {
                k: v.to_dict() for k, v in self.ink_gather_configs.items()
            },
            "default_gather_config": self.default_gather_config.to_dict(),
            "wash_cycles": self.wash_cycles,
            "travel_speed_mm_s": self.travel_speed_mm_s,
            "aspirate_speed_uL_s": self.aspirate_speed_uL_s,
            "dispense_speed_uL_s": self.dispense_speed_uL_s,
            "wash_time_per_cycle_s": self.wash_time_per_cycle_s,
            "pump_volume_overrides_uL": dict(self.pump_volume_overrides_uL),
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintExecutionConfig:
        cfg = cls()
        if "ink_swap" in data:
            cfg.ink_swap = InkSwapStrategy.from_dict(data["ink_swap"])
        else:
            # Legacy save (pre v7.5.0): rehydrate ink_swap toggles from
            # the old top-level use_waste / use_wash / use_buffer flags
            # via the property shims.
            if "use_waste" in data:
                cfg.use_waste = data["use_waste"]
            if "use_wash" in data:
                cfg.use_wash = data["use_wash"]
            if "use_buffer" in data:
                cfg.use_buffer = data["use_buffer"]
        if "z_travel" in data:
            cfg.z_travel = ZTravelConfig.from_dict(data["z_travel"])
        if "xy_travel" in data:
            cfg.xy_travel = XYTravelConfig.from_dict(data["xy_travel"])
        if "final_cleanup" in data:
            cfg.final_cleanup = FinalCleanupConfig.from_dict(data["final_cleanup"])
        if "ink_gather_configs" in data:
            cfg.ink_gather_configs = {
                k: InkGatherConfig.from_dict(v)
                for k, v in data["ink_gather_configs"].items()
            }
        if "default_gather_config" in data:
            cfg.default_gather_config = InkGatherConfig.from_dict(
                data["default_gather_config"])
        cfg.wash_cycles = data.get("wash_cycles", 3)
        cfg.travel_speed_mm_s = data.get("travel_speed_mm_s", 10.0)
        cfg.aspirate_speed_uL_s = data.get("aspirate_speed_uL_s", 1.0)
        cfg.dispense_speed_uL_s = data.get("dispense_speed_uL_s", 2.0)
        cfg.wash_time_per_cycle_s = data.get("wash_time_per_cycle_s", 5.0)
        # Accept either the new key (pump_volume_overrides_uL) or the
        # legacy max_ink_volume_uL key.
        cfg.pump_volume_overrides_uL = (
            data.get("pump_volume_overrides_uL")
            or data.get("max_ink_volume_uL", {})
        )
        return cfg

    @classmethod
    def from_preferences(cls, prefs) -> PrintExecutionConfig:
        """Migrate from old PlanPreferences to new PrintExecutionConfig."""
        cfg = cls()
        cfg.use_waste = getattr(prefs, 'use_waste',
                                getattr(prefs, 'waste_before_refill', True))
        cfg.use_wash = getattr(prefs, 'use_wash',
                               getattr(prefs, 'wash_after_refill', True))
        cfg.use_buffer = getattr(prefs, 'use_buffer',
                                 getattr(prefs, 'refill_buffer_after_waste', True))
        cfg.wash_cycles = getattr(prefs, 'wash_cycles', 3)
        cfg.travel_speed_mm_s = getattr(prefs, 'travel_speed_mm_s', 10.0)
        cfg.aspirate_speed_uL_s = getattr(prefs, 'aspirate_speed_uL_s', 1.0)
        cfg.dispense_speed_uL_s = getattr(prefs, 'dispense_speed_uL_s', 2.0)
        cfg.wash_time_per_cycle_s = getattr(prefs, 'wash_time_per_cycle_s', 5.0)
        cfg.pump_volume_overrides_uL = dict(getattr(prefs, 'max_ink_volume_uL', {}))
        return cfg


# ═══════════════════════════════════════════════════════════════════
#  Backward compat: PlanPreferences (thin wrapper)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PlanPreferences:
    """User-configurable preferences for plan generation.

    v7.2.9: Kept for backwards compatibility. New code should use
    PrintExecutionConfig directly.
    """
    max_ink_volume_uL: dict[str, float] = field(default_factory=dict)
    wash_after_refill: bool = True
    waste_before_refill: bool = True
    refill_buffer_after_waste: bool = True
    wash_cycles: int = 3
    travel_speed_mm_s: float = 10.0
    aspirate_speed_uL_s: float = 1.0
    dispense_speed_uL_s: float = 2.0
    wash_time_per_cycle_s: float = 5.0

    def to_dict(self) -> dict:
        return {
            "max_ink_volume_uL": dict(self.max_ink_volume_uL),
            "wash_after_refill": self.wash_after_refill,
            "waste_before_refill": self.waste_before_refill,
            "refill_buffer_after_waste": self.refill_buffer_after_waste,
            "wash_cycles": self.wash_cycles,
            "travel_speed_mm_s": self.travel_speed_mm_s,
            "aspirate_speed_uL_s": self.aspirate_speed_uL_s,
            "dispense_speed_uL_s": self.dispense_speed_uL_s,
            "wash_time_per_cycle_s": self.wash_time_per_cycle_s,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PlanPreferences:
        return cls(
            max_ink_volume_uL=data.get("max_ink_volume_uL", {}),
            wash_after_refill=data.get("wash_after_refill", True),
            waste_before_refill=data.get("waste_before_refill", True),
            refill_buffer_after_waste=data.get("refill_buffer_after_waste", True),
            wash_cycles=data.get("wash_cycles", 3),
            travel_speed_mm_s=data.get("travel_speed_mm_s", 10.0),
            aspirate_speed_uL_s=data.get("aspirate_speed_uL_s", 1.0),
            dispense_speed_uL_s=data.get("dispense_speed_uL_s", 2.0),
            wash_time_per_cycle_s=data.get("wash_time_per_cycle_s", 5.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  Plan Step (v7.2.9: richer fields)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PlanStep:
    """A single step in the print execution plan.

    v7.2.9: Added z_behavior, travel_mode, wait_for_confirm, extra_percent,
    ink_gather fields for comprehensive execution control.
    """
    step_type: PlanStepType
    description: str = ""
    target_wells: list[str] = field(default_factory=list)
    pump_id: str | None = None
    volume_uL: float = 0.0
    ink_name: str | None = None
    run_number: int = 1
    estimated_seconds: float = 0.0

    # v7.2.9: Z behavior for this step
    z_behavior: str = "default"         # "fast", "slow", "plunge", "default"
    wait_for_z_confirm: bool = True     # Wait for Z position confirmation
    fast_z_entry: bool = False          # Fast Z into well
    fast_z_exit: bool = True            # Fast Z out of well
    plunge_buffer_um: float = 500.0     # Buffer distance for plunge-in (microns)

    # v7.2.9: XY travel mode for this step
    travel_mode: str = "fast"           # "fast" or "slow"
    travel_speed_mm_s: float = 10.0     # XY travel speed for this step

    # v7.2.9: Ink gather details
    extra_percent: float = 0.0          # Extra ink % for gather steps
    max_pickup_uL: float = 0.0         # Max pickup for gather steps

    # v7.2.9: Sub-steps for compound operations (INK_SWAP, FINAL_CLEANUP)
    sub_steps: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        d = {
            "step_type": self.step_type.value,
            "description": self.description,
            "target_wells": list(self.target_wells),
            "pump_id": self.pump_id,
            "volume_uL": self.volume_uL,
            "ink_name": self.ink_name,
            "run_number": self.run_number,
            "estimated_seconds": self.estimated_seconds,
            "z_behavior": self.z_behavior,
            "wait_for_z_confirm": self.wait_for_z_confirm,
            "fast_z_entry": self.fast_z_entry,
            "fast_z_exit": self.fast_z_exit,
            "plunge_buffer_um": self.plunge_buffer_um,
            "travel_mode": self.travel_mode,
            "travel_speed_mm_s": self.travel_speed_mm_s,
            "extra_percent": self.extra_percent,
            "max_pickup_uL": self.max_pickup_uL,
            "sub_steps": list(self.sub_steps),
        }
        return d

    @classmethod
    def from_dict(cls, data: dict) -> PlanStep:
        # Handle both old and new step type values
        try:
            step_type = PlanStepType(data["step_type"])
        except ValueError:
            step_type = PlanStepType.LOAD_INK  # fallback
            logger.warning(f"Unknown step type: {data.get('step_type')}")
        return cls(
            step_type=step_type,
            description=data.get("description", ""),
            target_wells=data.get("target_wells", []),
            pump_id=data.get("pump_id"),
            volume_uL=data.get("volume_uL", 0.0),
            ink_name=data.get("ink_name"),
            run_number=data.get("run_number", 1),
            estimated_seconds=data.get("estimated_seconds", 0.0),
            z_behavior=data.get("z_behavior", "default"),
            wait_for_z_confirm=data.get("wait_for_z_confirm", True),
            fast_z_entry=data.get("fast_z_entry", False),
            fast_z_exit=data.get("fast_z_exit", True),
            plunge_buffer_um=data.get("plunge_buffer_um", 500.0),
            travel_mode=data.get("travel_mode", "fast"),
            travel_speed_mm_s=data.get("travel_speed_mm_s", 10.0),
            extra_percent=data.get("extra_percent", 0.0),
            max_pickup_uL=data.get("max_pickup_uL", 0.0),
            sub_steps=data.get("sub_steps", []),
        )

    @property
    def icon(self) -> str:
        return PLAN_STEP_ICONS.get(self.step_type, "\u2b1c")

    @property
    def color(self) -> str:
        return PLAN_STEP_COLORS.get(self.step_type, "#6c7086")


# ═══════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════

def _get_role_wells(well_model, role_str: str) -> list[str]:
    """Get wells by role string, with fallbacks for different model versions."""
    if hasattr(well_model, 'get_wells_by_role_str'):
        return well_model.get_wells_by_role_str(role_str)
    if hasattr(well_model, 'get_wells_by_role'):
        try:
            from SupportClasses.PhysicalModels import WellRole
            role = WellRole(role_str)
            return well_model.get_wells_by_role(role)
        except (ValueError, ImportError):
            pass
    result = []
    if hasattr(well_model, 'assignments'):
        for name, wa in well_model.assignments.items():
            role_val = wa.role.value if hasattr(wa.role, 'value') else str(wa.role)
            if role_val == role_str:
                result.append(name)
    return result


# ═══════════════════════════════════════════════════════════════════
#  Print Plan of Action (v7.2.9: uses PrintExecutionConfig)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PrintPlanOfAction:
    """Complete execution plan for a bioprinting job.

    v7.2.9: Now uses PrintExecutionConfig for comprehensive control
    over all aspects of plan generation. The old 'preferences' field
    is kept for backwards compatibility but new code should use
    'execution_config'.
    """
    steps: list[PlanStep] = field(default_factory=list)
    total_runs: int = 0
    preferences: PlanPreferences = field(default_factory=PlanPreferences)
    execution_config: PrintExecutionConfig = field(
        default_factory=PrintExecutionConfig)
    ink_consumption_uL: dict[str, float] = field(default_factory=dict)
    total_print_wells: int = 0
    total_ink_volume_uL: float = 0.0
    estimated_total_seconds: float = 0.0

    @classmethod
    def generate_plan(
        cls,
        hw_config,
        well_model,
        preferences: PlanPreferences | None = None,
        execution_config: PrintExecutionConfig | None = None,
    ) -> PrintPlanOfAction:
        """Generate a complete execution plan.

        Args:
            hw_config: HardwareConfig
            well_model: WellSetupModel
            preferences: Legacy PlanPreferences (optional, for backwards compat)
            execution_config: New PrintExecutionConfig (preferred)

        If execution_config is provided, it takes priority.
        If only preferences is provided, it is migrated to an execution_config.
        """
        if execution_config is None:
            if preferences is not None:
                execution_config = PrintExecutionConfig.from_preferences(preferences)
            else:
                execution_config = PrintExecutionConfig()

        if preferences is None:
            preferences = PlanPreferences()

        plan = cls(preferences=preferences, execution_config=execution_config)
        plan._compute(hw_config, well_model)
        return plan

    # ══════════════════════════════════════════════════════════════
    #  Core Plan Generation (v7.2.9)
    # ══════════════════════════════════════════════════════════════

    def _compute(self, hw_config, well_model) -> None:
        """Generate comprehensive plan using execution_config."""
        self.steps.clear()
        self.ink_consumption_uL.clear()
        cfg = self.execution_config

        print_wells = _get_role_wells(well_model, "print")
        if not print_wells:
            logger.warning("No print wells assigned")
            return

        self.total_print_wells = len(print_wells)
        pump_ink_needs = self._compute_ink_needs(hw_config, well_model, print_wells)

        if not pump_ink_needs:
            default_pump = "P1"
            enabled = getattr(hw_config, 'enabled_pump_ids', [])
            if enabled:
                default_pump = enabled[0]
            estimated_vol = max(len(print_wells) * 2.0, 10.0)
            pump_ink_needs = {default_pump: ("unknown", estimated_vol)}
            logger.info(
                f"No ink info -- defaulting to {default_pump}: "
                f"{estimated_vol:.1f}uL")

        # Determine max per run (applying per-ink gather configs)
        max_per_run = {}
        for pump_id, (ink_name, total_needed) in pump_ink_needs.items():
            if pump_id in cfg.pump_volume_overrides_uL:
                max_per_run[pump_id] = cfg.pump_volume_overrides_uL[pump_id]
            else:
                pcfg = hw_config.pumps.get(pump_id)
                if pcfg and pcfg.syringe:
                    max_per_run[pump_id] = pcfg.syringe.volume_uL
                else:
                    max_per_run[pump_id] = 100.0

        # Compute number of runs needed
        runs_needed = 1
        for pump_id, (ink_name, total_needed) in pump_ink_needs.items():
            max_vol = max_per_run.get(pump_id, 100.0)
            if max_vol > 0:
                runs_needed = max(runs_needed, math.ceil(total_needed / max_vol))

        self.total_runs = runs_needed
        self.ink_consumption_uL = {
            pid: vol for pid, (_, vol) in pump_ink_needs.items()}
        self.total_ink_volume_uL = sum(self.ink_consumption_uL.values())

        # Split wells into run groups
        wells_per_run = math.ceil(len(print_wells) / runs_needed)
        run_groups = []
        for i in range(runs_needed):
            start = i * wells_per_run
            end = min(start + wells_per_run, len(print_wells))
            if start < end:
                run_groups.append(print_wells[start:end])

        ink_wells = _get_role_wells(well_model, "ink")
        wash_wells = _get_role_wells(well_model, "wash")
        waste_wells = _get_role_wells(well_model, "waste")
        buffer_wells = _get_role_wells(well_model, "buffer")

        # Generate steps for each run
        for run_idx, well_group in enumerate(run_groups):
            run_num = run_idx + 1

            # Compute ink needed for this run (per pump)
            run_ink = {}
            for pump_id, (ink_name, total) in pump_ink_needs.items():
                run_ink[pump_id] = (
                    ink_name,
                    min(total / runs_needed,
                        max_per_run.get(pump_id, 100.0)),
                )

            # ── Service sequence ──────────────────────────────────
            self._add_service_steps(
                run_num, run_ink, hw_config, cfg,
                ink_wells, wash_wells, waste_wells, buffer_wells,
            )

            # ── Travel to first print well ────────────────────────
            if cfg.xy_travel.fast_xy_travel:
                self.steps.append(PlanStep(
                    step_type=PlanStepType.MOVE_SAFE_Z,
                    description=f"Raise to safe Z ({cfg.z_travel.safe_z_mm:.1f}mm)",
                    run_number=run_num,
                    wait_for_z_confirm=cfg.z_travel.wait_for_z_confirm,
                ))

                well_str = well_group[0] if well_group else "?"
                self.steps.append(PlanStep(
                    step_type=PlanStepType.TRAVEL_XY,
                    description=f"Fast travel to {well_str}",
                    target_wells=[well_str] if well_str != "?" else [],
                    run_number=run_num,
                    travel_mode="fast",
                    travel_speed_mm_s=cfg.xy_travel.fast_xy_speed_mm_s,
                ))

            # ── Print step ────────────────────────────────────────
            well_str = ", ".join(well_group[:3])
            if len(well_group) > 3:
                well_str += "..."
            self.steps.append(PlanStep(
                step_type=PlanStepType.PRINT,
                description=(
                    f"Print wells {well_str} "
                    f"(run {run_num}/{runs_needed})"),
                target_wells=list(well_group),
                run_number=run_num,
                # Propagate Z behavior to print step
                z_behavior="plunge" if cfg.z_travel.use_plunge_buffer else "default",
                wait_for_z_confirm=cfg.z_travel.wait_for_z_confirm,
                fast_z_entry=cfg.z_travel.fast_z_enter_well,
                fast_z_exit=cfg.z_travel.fast_z_exit_well,
                plunge_buffer_um=cfg.z_travel.plunge_buffer_um,
                travel_mode=(
                    "fast" if cfg.xy_travel.fast_xy_travel else "slow"),
                travel_speed_mm_s=cfg.xy_travel.fast_xy_speed_mm_s,
            ))

        # ── Final cleanup ─────────────────────────────────────────
        if cfg.final_cleanup.enabled:
            sub_steps = []
            if cfg.final_cleanup.do_waste:
                sub_steps.append("waste")
            if cfg.final_cleanup.do_wash:
                sub_steps.append("wash")
            if cfg.final_cleanup.do_dry_run:
                sub_steps.append("dry_run")

            self.steps.append(PlanStep(
                step_type=PlanStepType.FINAL_CLEANUP,
                description=(
                    f"Final cleanup: "
                    f"{', '.join(sub_steps) if sub_steps else 'none'}"),
                run_number=runs_needed,
                sub_steps=sub_steps,
                target_wells=(
                    (waste_wells[:1] if cfg.final_cleanup.do_waste else [])
                    + (wash_wells[:1] if cfg.final_cleanup.do_wash else [])),
            ))

        # ── Return home ───────────────────────────────────────────
        self.steps.append(PlanStep(
            step_type=PlanStepType.RETURN_HOME,
            description="Return to home position",
            run_number=runs_needed,
            wait_for_z_confirm=cfg.z_travel.wait_for_z_confirm,
        ))

        self._estimate_times()
        logger.info(
            f"Plan: {len(self.steps)} steps, {self.total_runs} runs, "
            f"{self.total_print_wells} wells, "
            f"{self.total_ink_volume_uL:.1f} uL")

    def _compute_ink_needs(self, hw_config, well_model, print_wells):
        """Compute ink needed per pump.

        Returns dict[pump_id] -> (ink_name, total_volume_uL).
        """
        pump_ink_needs: dict[str, tuple[str, float]] = {}
        for well_name in print_wells:
            assignment = well_model.get_assignment(well_name)
            if assignment is None:
                continue
            for _coll in assignment.print_collections:
                ink_per_coll = 2.0
                enabled = hw_config.enabled_pump_ids
                if enabled:
                    pid = enabled[0]
                    pcfg = hw_config.pumps.get(pid)
                    ink_name = (
                        pcfg.ink.name if pcfg and pcfg.ink else "unknown")
                    existing_name, existing_vol = pump_ink_needs.get(
                        pid, (ink_name, 0.0))
                    pump_ink_needs[pid] = (
                        existing_name, existing_vol + ink_per_coll)
        return pump_ink_needs

    def _add_service_steps(self, run_num, run_ink_per_pump, hw_config, cfg,
                           ink_wells, wash_wells, waste_wells, buffer_wells):
        """v7.2.9: Generate service steps using PrintExecutionConfig.

        For each run, generates:
        1. WASTE -- purge syringe
        2. WASH -- clean needle
        3. REFILL_BUFFER -- load buffer layer
        4. WASH -- clean after buffer
        5. GATHER_INK -- load ink with per-ink extra %
        """
        z = cfg.z_travel

        # 1. WASTE -- purge syringe
        if cfg.use_waste and waste_wells:
            for pump_id in run_ink_per_pump:
                self.steps.append(PlanStep(
                    step_type=PlanStepType.WASTE,
                    description=f"Purge {pump_id} at {waste_wells[0]}",
                    target_wells=[waste_wells[0]],
                    pump_id=pump_id,
                    run_number=run_num,
                    wait_for_z_confirm=z.wait_for_z_confirm,
                    fast_z_entry=z.fast_z_enter_well,
                    fast_z_exit=z.fast_z_exit_well,
                    plunge_buffer_um=z.plunge_buffer_um,
                ))

        # 2. WASH -- clean needle
        if cfg.use_wash and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Wash needle at {wash_wells[0]}",
                target_wells=[wash_wells[0]],
                run_number=run_num,
                wait_for_z_confirm=z.wait_for_z_confirm,
                fast_z_entry=z.fast_z_enter_well,
                fast_z_exit=z.fast_z_exit_well,
            ))

        # 3. BUFFER -- load buffer layer
        if cfg.use_buffer and buffer_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.REFILL_BUFFER,
                description=f"Load buffer from {buffer_wells[0]}",
                target_wells=[buffer_wells[0]],
                run_number=run_num,
                wait_for_z_confirm=z.wait_for_z_confirm,
                fast_z_entry=z.fast_z_enter_well,
                fast_z_exit=z.fast_z_exit_well,
            ))

        # 4. WASH again -- clean after buffer
        if cfg.use_wash and cfg.use_buffer and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Post-buffer wash at {wash_wells[0]}",
                target_wells=[wash_wells[0]],
                run_number=run_num,
                wait_for_z_confirm=z.wait_for_z_confirm,
                fast_z_entry=z.fast_z_enter_well,
                fast_z_exit=z.fast_z_exit_well,
            ))

        # 5. GATHER_INK -- aspirate ink for this run with per-ink config
        for pump_id, (ink_name, base_volume) in run_ink_per_pump.items():
            if base_volume <= 0:
                continue

            # Get per-ink gather config
            gather = cfg.get_ink_gather(ink_name)
            pcfg = hw_config.pumps.get(pump_id)
            syringe_cap = (
                pcfg.syringe.volume_uL
                if pcfg and pcfg.syringe else 100.0)
            actual_volume = gather.effective_volume(base_volume, syringe_cap)

            target = ink_wells[0] if ink_wells else "?"
            extra_pct = gather.extra_percent

            self.steps.append(PlanStep(
                step_type=PlanStepType.GATHER_INK,
                description=(
                    f"Gather {actual_volume:.1f} uL {ink_name} "
                    f"into {pump_id} (+{extra_pct:.0f}%)"),
                target_wells=[target] if target != "?" else [],
                pump_id=pump_id,
                volume_uL=actual_volume,
                ink_name=ink_name,
                run_number=run_num,
                extra_percent=extra_pct,
                max_pickup_uL=gather.max_pickup_uL,
                wait_for_z_confirm=z.wait_for_z_confirm,
                fast_z_entry=z.fast_z_enter_well,
                fast_z_exit=z.fast_z_exit_well,
                plunge_buffer_um=z.plunge_buffer_um,
            ))

    def _estimate_times(self):
        """Estimate execution time for each step."""
        cfg = self.execution_config
        total = 0.0
        for step in self.steps:
            st = step.step_type
            if st == PlanStepType.GATHER_INK:
                t = 3.0 + (step.volume_uL /
                           max(cfg.aspirate_speed_uL_s, 0.01))
            elif st == PlanStepType.LOAD_INK:
                t = 3.0 + (step.volume_uL /
                           max(cfg.aspirate_speed_uL_s, 0.01))
            elif st == PlanStepType.WASH:
                t = 3.0 + (cfg.wash_cycles * cfg.wash_time_per_cycle_s)
            elif st == PlanStepType.WASTE:
                t = 5.0
            elif st == PlanStepType.REFILL_BUFFER:
                t = 8.0
            elif st == PlanStepType.PRINT:
                t = len(step.target_wells) * 30.0
            elif st == PlanStepType.RETURN_HOME:
                t = 5.0
            elif st == PlanStepType.TRAVEL_XY:
                t = 2.0  # Rough estimate for fast XY travel
            elif st == PlanStepType.MOVE_SAFE_Z:
                t = 1.5
            elif st == PlanStepType.INK_SWAP:
                t = 30.0  # Full ink swap sequence
            elif st == PlanStepType.FINAL_CLEANUP:
                t = 5.0
                if "waste" in step.sub_steps:
                    t += 5.0
                if "wash" in step.sub_steps:
                    t += cfg.final_cleanup.wash_cycles * cfg.final_cleanup.wash_time_per_cycle_s
                if "dry_run" in step.sub_steps:
                    t += 3.0
            else:
                t = 1.0
            step.estimated_seconds = t
            total += t
        self.estimated_total_seconds = total

    # ══════════════════════════════════════════════════════════════
    #  Validation
    # ══════════════════════════════════════════════════════════════

    def validate(self, hw_config=None, well_model=None):
        """Validate plan for completeness and correctness."""
        issues = []
        if not self.steps:
            issues.append("Plan is empty")
            return (False, issues)

        has_print = any(
            s.step_type == PlanStepType.PRINT for s in self.steps)
        if not has_print:
            issues.append("Plan has no print steps")

        # Check ink steps have valid wells
        ink_step_types = (PlanStepType.LOAD_INK, PlanStepType.GATHER_INK)
        for step in self.steps:
            if step.step_type in ink_step_types:
                if not step.target_wells or step.target_wells[0] == "?":
                    issues.append(
                        f"Ink load for {step.pump_id} ({step.ink_name}) "
                        f"has no target ink well")

        # Check required service wells exist
        if well_model is not None:
            needs = {
                "wash": any(s.step_type == PlanStepType.WASH
                            for s in self.steps),
                "waste": any(s.step_type == PlanStepType.WASTE
                             for s in self.steps),
                "buffer": any(s.step_type == PlanStepType.REFILL_BUFFER
                              for s in self.steps),
            }
            # Also check final cleanup needs
            for s in self.steps:
                if s.step_type == PlanStepType.FINAL_CLEANUP:
                    if "waste" in s.sub_steps:
                        needs["waste"] = True
                    if "wash" in s.sub_steps:
                        needs["wash"] = True

            for role_str, needed in needs.items():
                if needed and not _get_role_wells(well_model, role_str):
                    issues.append(
                        f"Plan requires {role_str} wells but none assigned")

        # Check syringe capacity
        if hw_config is not None:
            for step in self.steps:
                if step.step_type in ink_step_types and step.pump_id:
                    pcfg = hw_config.pumps.get(step.pump_id)
                    if pcfg and pcfg.syringe:
                        if step.volume_uL > pcfg.syringe.volume_uL:
                            issues.append(
                                f"Ink load {step.volume_uL:.1f} uL for "
                                f"{step.pump_id} exceeds syringe "
                                f"({pcfg.syringe.volume_uL:.1f} uL)")

        return (len(issues) == 0, issues)

    # ══════════════════════════════════════════════════════════════
    #  Display & Serialization
    # ══════════════════════════════════════════════════════════════

    def summary(self) -> str:
        minutes = self.estimated_total_seconds / 60.0
        return (
            f"Total runs: {self.total_runs} | "
            f"Steps: {len(self.steps)} | "
            f"Print wells: {self.total_print_wells} | "
            f"Total ink: {self.total_ink_volume_uL:.1f} uL | "
            f"Est. time: {minutes:.1f} min"
        )

    def step_summary_lines(self) -> list[str]:
        return [
            f"{i}. {step.icon} {step.description}"
            for i, step in enumerate(self.steps, 1)
        ]

    def to_dict(self) -> dict:
        return {
            "steps": [s.to_dict() for s in self.steps],
            "total_runs": self.total_runs,
            "preferences": self.preferences.to_dict(),
            "execution_config": self.execution_config.to_dict(),
            "ink_consumption_uL": dict(self.ink_consumption_uL),
            "total_print_wells": self.total_print_wells,
            "total_ink_volume_uL": self.total_ink_volume_uL,
            "estimated_total_seconds": self.estimated_total_seconds,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintPlanOfAction:
        exec_cfg = PrintExecutionConfig()
        if "execution_config" in data:
            exec_cfg = PrintExecutionConfig.from_dict(
                data["execution_config"])
        return cls(
            steps=[PlanStep.from_dict(s) for s in data.get("steps", [])],
            total_runs=data.get("total_runs", 0),
            preferences=PlanPreferences.from_dict(
                data.get("preferences", {})),
            execution_config=exec_cfg,
            ink_consumption_uL=data.get("ink_consumption_uL", {}),
            total_print_wells=data.get("total_print_wells", 0),
            total_ink_volume_uL=data.get("total_ink_volume_uL", 0.0),
            estimated_total_seconds=data.get("estimated_total_seconds", 0.0),
        )


# ═══════════════════════════════════════════════════════════════════
#  Validation Helper
# ═══════════════════════════════════════════════════════════════════

def validate_well_setup(hw_config, well_model, plan=None):
    """
    Comprehensive validation before sending to monitor.
    Returns (is_valid, list_of_issues).
    """
    issues = []

    # 1. Print wells
    print_wells = _get_role_wells(well_model, "print")
    if not print_wells:
        issues.append("No print wells assigned on the plate")
    else:
        for name in print_wells:
            wa = well_model.get_assignment(name)
            if wa and not wa.print_collections:
                issues.append(f"Print well {name} has no print files assigned")

    # 2. Ink assignments (incremental mode)
    if hw_config is not None:
        try:
            from SupportClasses.PhysicalModels import PrintingMode
            for pid, pcfg in hw_config.pumps.items():
                if not pcfg.enabled:
                    continue
                if pcfg.printing_mode == PrintingMode.INCREMENTAL and pcfg.ink:
                    ink_name = pcfg.ink.name
                    ink_wells = _get_role_wells(well_model, "ink")
                    has_match = any(
                        getattr(well_model.get_assignment(iw),
                                'ink_name', None) == ink_name
                        for iw in ink_wells
                    )
                    if not has_match:
                        issues.append(
                            f"Pump {pid} uses '{ink_name}' (incremental) "
                            f"but no ink well assigned for it")
        except ImportError:
            pass

    # 3. Service wells
    if plan is not None:
        needs = {
            "wash": any(s.step_type == PlanStepType.WASH
                        for s in plan.steps),
            "waste": any(s.step_type == PlanStepType.WASTE
                         for s in plan.steps),
            "buffer": any(s.step_type == PlanStepType.REFILL_BUFFER
                          for s in plan.steps),
        }
        # Also check final cleanup
        for s in plan.steps:
            if s.step_type == PlanStepType.FINAL_CLEANUP:
                if "waste" in s.sub_steps:
                    needs["waste"] = True
                if "wash" in s.sub_steps:
                    needs["wash"] = True

        for role_str, needed in needs.items():
            if needed and not _get_role_wells(well_model, role_str):
                issues.append(
                    f"Plan requires {role_str} wells but none assigned")

    # 4. Pump-needle mapping
    if hw_config is not None and hw_config.needle is not None:
        num_ch = hw_config.needle.num_channels
        enabled = hw_config.enabled_pump_ids
        if num_ch > 0 and enabled:
            mapped = hw_config.needle_channel_pump_map
            if len(mapped) != num_ch:
                issues.append(
                    f"Needle has {num_ch} channel(s) but "
                    f"{len(mapped)} mapped")

    # 5. Syringe capacity
    if plan is not None and hw_config is not None:
        ink_step_types = (PlanStepType.LOAD_INK, PlanStepType.GATHER_INK)
        for step in plan.steps:
            if step.step_type in ink_step_types and step.pump_id:
                pcfg = hw_config.pumps.get(step.pump_id)
                if pcfg and pcfg.syringe:
                    if step.volume_uL > pcfg.syringe.volume_uL:
                        issues.append(
                            f"Run {step.run_number}: "
                            f"{step.volume_uL:.1f} uL "
                            f"for {step.pump_id} exceeds syringe "
                            f"({pcfg.syringe.volume_uL:.1f} uL)")

    # 6. Plan validation
    if plan is None:
        issues.append("No execution plan generated")
    else:
        plan_ok, plan_issues = plan.validate(hw_config, well_model)
        if not plan_ok:
            issues.extend(plan_issues)

    return (len(issues) == 0, issues)


# ═══════════════════════════════════════════════════════════════════
# v7.2.6: Plan -> PrintCommand Execution Bridge
# ═══════════════════════════════════════════════════════════════════

def _signed_well_xy(plate, name, settings):
    """v7.5.x: plate-local well centre (A1-relative mm) mapped onto stage axes
    via the per-machine ``settings.plate_axis_sign`` so a GEOMETRIC well centre
    reaches the physically-correct well on a 180°-mounted stage (ME3B V1)."""
    wx, wy = plate.get_well_position(name)
    try:
        _s = getattr(settings, "plate_axis_sign", (1.0, 1.0))
        sx, sy = float(_s[0]), float(_s[1])
    except Exception:
        sx, sy = 1.0, 1.0
    return (sx * wx, sy * wy)


def _find_service_well(well_model, plate, role_value: str):
    """Find first well with given role. Returns (name, x, y) or None.

    NOTE: x, y are PLATE-LOCAL (A1-relative mm). Callers that turn these into
    a stage move must map them with the per-machine sign — re-resolve via
    :func:`_signed_well_xy` (the raw return is kept for back-compat)."""
    if well_model is None or plate is None:
        return None
    assignments = getattr(well_model, 'assignments', {})
    for name, assignment in assignments.items():
        r = getattr(assignment, 'role', None)
        if r is not None and getattr(r, 'value', None) == role_value:
            try:
                x, y = plate.get_well_position(name)
                return (name, x, y)
            except Exception:
                continue
    return None


def _waste_commands(step, well_model, plate, settings):
    """Generate waste-dispense commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "waste")
    if not well:
        return [PrintCommand(
            type=CommandType.COMMENT, label="SKIP: No waste well")]
    name, x, y = well
    x, y = _signed_well_xy(plate, name, settings)  # map plate-local → stage axes
    pump = (step.pump_id
            or getattr(settings, 'active_pump', 'P1') or 'P1')
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    dispense_vol = 5.0
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Waste: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY,
                     params={"x": x, "y": y},
                     label=f"Travel to waste well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN,
                     params={"feedrate": z_fr},
                     label="Lower to waste depth"),
        PrintCommand(type=CommandType.DISPENSE,
                     params={"pump": pump, "amount": dispense_vol,
                             "feedrate": p_fr},
                     label=f"Dispense {dispense_vol:.1f} into waste"),
        PrintCommand(type=CommandType.DWELL,
                     params={"seconds": 0.5}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise from waste"),
    ]


def _wash_commands(step, well_model, plate, settings):
    """Generate wash commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "wash")
    if not well:
        return [PrintCommand(
            type=CommandType.COMMENT, label="SKIP: No wash well")]
    name, x, y = well
    x, y = _signed_well_xy(plate, name, settings)  # map plate-local → stage axes
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Wash: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY,
                     params={"x": x, "y": y},
                     label=f"Travel to wash well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN,
                     params={"feedrate": z_fr},
                     label="Lower into wash"),
        PrintCommand(type=CommandType.DWELL,
                     params={"seconds": 5.0}, label="Wash soak"),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise from wash"),
    ]


def _buffer_commands(step, well_model, plate, settings):
    """Generate buffer commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "buffer")
    if not well:
        return [PrintCommand(
            type=CommandType.COMMENT, label="SKIP: No buffer well")]
    name, x, y = well
    x, y = _signed_well_xy(plate, name, settings)  # map plate-local → stage axes
    pump = (step.pump_id
            or getattr(settings, 'active_pump', 'P1') or 'P1')
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Buffer: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr}, label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY,
                     params={"x": x, "y": y},
                     label=f"Travel to buffer {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN,
                     params={"feedrate": z_fr},
                     label="Lower into buffer"),
        PrintCommand(type=CommandType.DISPENSE,
                     params={"pump": pump, "amount": -5.0,
                             "feedrate": p_fr},
                     label="Aspirate buffer"),
        PrintCommand(type=CommandType.DWELL,
                     params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise from buffer"),
    ]


def _load_ink_commands(step, well_model, plate, settings):
    """Generate ink loading commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "ink")
    if not well:
        return [PrintCommand(
            type=CommandType.COMMENT, label="SKIP: No ink well")]
    name, x, y = well
    x, y = _signed_well_xy(plate, name, settings)  # map plate-local → stage axes
    pump = (step.pump_id
            or getattr(settings, 'active_pump', 'P1') or 'P1')
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    volume = step.volume_uL if step.volume_uL > 0 else 50.0
    ink_name = step.ink_name or "?"
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Load Ink: {ink_name} ({volume:.1f}uL) "
                           f"into {pump} =="),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr}, label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY,
                     params={"x": x, "y": y},
                     label=f"Travel to ink well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN,
                     params={"feedrate": z_fr},
                     label="Lower into ink"),
        PrintCommand(type=CommandType.DISPENSE,
                     params={"pump": pump, "amount": -volume,
                             "feedrate": p_fr},
                     label=f"Aspirate {volume:.1f}uL {ink_name}"),
        PrintCommand(type=CommandType.DWELL,
                     params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP,
                     params={"feedrate": z_fr},
                     label="Raise from ink"),
    ]


def plan_to_commands(plan, well_model, plate, path_points, settings,
                     hw_config=None):
    """Convert PrintPlanOfAction into executable PrintJob.

    v7.2.9: Handles new step types (GATHER_INK, FINAL_CLEANUP,
    TRAVEL_XY, MOVE_SAFE_Z, INK_SWAP).
    """
    from SupportClasses.PrintManager import (
        PrintJob, PrintCommand, CommandType, build_well_plate_job,
    )

    steps = getattr(plan, 'steps', [])
    if not steps:
        logger.warning("plan_to_commands: empty plan")
        return None

    all_commands = []
    prev_pump = None

    # Track fluid balance per pump
    fluid_balance: dict[str, float] = {
        "P1": 0.0, "P2": 0.0, "P3": 0.0}

    for step in steps:
        stype = getattr(step, 'step_type', None)
        if stype is None:
            continue

        if stype == PlanStepType.WASTE:
            pump = step.pump_id or "P1"
            all_commands.extend(
                _waste_commands(step, well_model, plate, settings))
            fluid_balance[pump] = max(
                0, fluid_balance.get(pump, 0) - 5.0)

        elif stype == PlanStepType.WASH:
            all_commands.extend(
                _wash_commands(step, well_model, plate, settings))

        elif stype == PlanStepType.REFILL_BUFFER:
            pump = step.pump_id or "P1"
            all_commands.extend(
                _buffer_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + 5.0

        elif stype in (PlanStepType.LOAD_INK, PlanStepType.GATHER_INK):
            pump = step.pump_id or "P1"
            vol = step.volume_uL if step.volume_uL > 0 else 50.0
            all_commands.extend(
                _load_ink_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + vol

        elif stype == PlanStepType.MOVE_SAFE_Z:
            z_fr = getattr(settings, 'z_feedrate', 60.0)
            all_commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP,
                params={"feedrate": z_fr},
                label=f"Raise to safe Z"))

        elif stype == PlanStepType.TRAVEL_XY:
            # Fast XY travel -- resolve well position
            if step.target_wells:
                try:
                    x, y = _signed_well_xy(plate, step.target_wells[0], settings)
                    all_commands.append(PrintCommand(
                        type=CommandType.MOVE_XY,
                        params={"x": x, "y": y},
                        label=f"Fast travel to {step.target_wells[0]}"))
                except Exception:
                    all_commands.append(PrintCommand(
                        type=CommandType.COMMENT,
                        label=f"SKIP: Can't resolve "
                              f"{step.target_wells[0]}"))

        elif stype == PlanStepType.FINAL_CLEANUP:
            z_fr = getattr(settings, 'z_feedrate', 60.0)
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label="== Final Cleanup =="))
            if "waste" in step.sub_steps:
                # Create a temporary step for waste commands
                waste_step = PlanStep(
                    step_type=PlanStepType.WASTE,
                    pump_id=prev_pump or "P1")
                all_commands.extend(
                    _waste_commands(
                        waste_step, well_model, plate, settings))
            if "wash" in step.sub_steps:
                wash_step = PlanStep(step_type=PlanStepType.WASH)
                all_commands.extend(
                    _wash_commands(
                        wash_step, well_model, plate, settings))

        elif stype == PlanStepType.PRINT:
            target_wells = getattr(step, 'target_wells', [])
            if not target_wells:
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT, label="SKIP: No wells"))
                continue

            well_positions = []
            for wn in target_wells:
                try:
                    x, y = _signed_well_xy(plate, wn, settings)
                    well_positions.append((wn, x, y))
                except Exception:
                    continue

            if not well_positions:
                continue

            pump = (step.pump_id
                    or getattr(settings, 'active_pump', 'P1')
                    or 'P1')
            flow = getattr(settings, 'flow_rate', 0.01) or 0.01

            if prev_pump is not None and pump != prev_pump:
                all_commands.append(PrintCommand(
                    type=CommandType.SWITCH_PUMP,
                    params={"pump": pump},
                    label=f"Switch to {pump}"))
            prev_pump = pump

            # Estimate ink needed
            path_length = 0.0
            if len(path_points) >= 2:
                for i in range(1, len(path_points)):
                    dx = path_points[i][0] - path_points[i-1][0]
                    dy = path_points[i][1] - path_points[i-1][1]
                    path_length += math.sqrt(dx*dx + dy*dy)
            ink_per_well = (path_length * flow
                            * getattr(settings, 'num_layers', 1))
            ink_needed = ink_per_well * len(well_positions)

            balance = fluid_balance.get(pump, 0)
            if balance < ink_needed and ink_needed > 0:
                shortfall = ink_needed - balance
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=(f"WARNING: {pump} needs {ink_needed:.1f}uL "
                           f"but only {balance:.1f}uL loaded")))
                logger.warning(
                    f"Pump {pump}: needs {ink_needed:.1f}uL, "
                    f"has {balance:.1f}uL "
                    f"(shortfall {shortfall:.1f}uL)")

            run_num = getattr(step, 'run_number', '?')
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=(f"== Print Run {run_num}: "
                       f"{len(well_positions)} wells, {pump} ==")))

            try:
                sub_job = build_well_plate_job(
                    well_positions=well_positions,
                    path_points=path_points,
                    settings=settings,
                    pump=pump,
                    flow_rate=flow,
                    job_name=f"Run {run_num}",
                )
                all_commands.extend(sub_job.commands)
                fluid_balance[pump] = (
                    fluid_balance.get(pump, 0) - ink_needed)
            except Exception as exc:
                logger.error(
                    f"build_well_plate_job run {run_num}: {exc}")
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"ERROR: Run {run_num} failed: {exc}"))

        elif stype == PlanStepType.RETURN_HOME:
            z_fr = getattr(settings, 'z_feedrate', 60.0)
            all_commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP,
                params={"feedrate": z_fr},
                label="Final: raise"))
            all_commands.append(PrintCommand(
                type=CommandType.HOME_XY,
                label="Return home"))
        else:
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=f"Unknown: {stype}"))

    if not all_commands:
        return None

    tw = getattr(plan, 'total_print_wells', '?')
    tr = getattr(plan, 'total_runs', '?')
    return PrintJob(
        name=f"Plan: {tw} wells, {tr} run(s)",
        description="Generated from PrintPlanOfAction v7.2.9",
        settings=settings,
        commands=all_commands,
    )
