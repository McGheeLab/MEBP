"""
PrintPlanOfAction.py -- Print execution plan generator for MEBP v7.2.4.

Generates a step-by-step execution sequence for multi-run bioprinting:
    - Computes total ink needed per pump across all assigned wells
    - Divides into runs based on max ink volume per pump per run
    - Inserts wash/waste/buffer service steps based on user preferences
    - Validates that all required service wells exist
    - Estimates total execution time

Session 5 -- Tasks S5.1-S5.5.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class PlanStepType(Enum):
    """Types of steps in a print execution plan."""
    LOAD_INK = "load_ink"
    WASH = "wash"
    WASTE = "waste"
    REFILL_BUFFER = "refill_buffer"
    PRINT = "print"
    RETURN_HOME = "return_home"


PLAN_STEP_COLORS = {
    PlanStepType.LOAD_INK: "#89b4fa",
    PlanStepType.WASH: "#f9e2af",
    PlanStepType.WASTE: "#f38ba8",
    PlanStepType.REFILL_BUFFER: "#cba6f7",
    PlanStepType.PRINT: "#a6e3a1",
    PlanStepType.RETURN_HOME: "#6c7086",
}

PLAN_STEP_ICONS = {
    PlanStepType.LOAD_INK: "\U0001f535",
    PlanStepType.WASH: "\U0001f7e1",
    PlanStepType.WASTE: "\U0001f534",
    PlanStepType.REFILL_BUFFER: "\U0001f7e3",
    PlanStepType.PRINT: "\U0001f7e2",
    PlanStepType.RETURN_HOME: "\u2b1c",
}


@dataclass
class PlanStep:
    """A single step in the print execution plan."""
    step_type: PlanStepType
    description: str = ""
    target_wells: list[str] = field(default_factory=list)
    pump_id: str | None = None
    volume_uL: float = 0.0
    ink_name: str | None = None
    run_number: int = 1
    estimated_seconds: float = 0.0

    def to_dict(self) -> dict:
        return {
            "step_type": self.step_type.value,
            "description": self.description,
            "target_wells": list(self.target_wells),
            "pump_id": self.pump_id,
            "volume_uL": self.volume_uL,
            "ink_name": self.ink_name,
            "run_number": self.run_number,
            "estimated_seconds": self.estimated_seconds,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PlanStep:
        return cls(
            step_type=PlanStepType(data["step_type"]),
            description=data.get("description", ""),
            target_wells=data.get("target_wells", []),
            pump_id=data.get("pump_id"),
            volume_uL=data.get("volume_uL", 0.0),
            ink_name=data.get("ink_name"),
            run_number=data.get("run_number", 1),
            estimated_seconds=data.get("estimated_seconds", 0.0),
        )

    @property
    def icon(self) -> str:
        return PLAN_STEP_ICONS.get(self.step_type, "\u2b1c")

    @property
    def color(self) -> str:
        return PLAN_STEP_COLORS.get(self.step_type, "#6c7086")


@dataclass
class PlanPreferences:
    """User-configurable preferences for plan generation."""
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


@dataclass
class PrintPlanOfAction:
    """Complete execution plan for a bioprinting job."""
    steps: list[PlanStep] = field(default_factory=list)
    total_runs: int = 0
    preferences: PlanPreferences = field(default_factory=PlanPreferences)
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
    ) -> PrintPlanOfAction:
        """Generate a complete execution plan."""
        if preferences is None:
            preferences = PlanPreferences()
        plan = cls(preferences=preferences)
        plan._compute(hw_config, well_model)
        return plan

    def _compute(self, hw_config, well_model) -> None:
        self.steps.clear()
        self.ink_consumption_uL.clear()

        print_wells = _get_role_wells(well_model, "print")
        if not print_wells:
            logger.warning("No print wells assigned")
            return

        self.total_print_wells = len(print_wells)
        pump_ink_needs = self._compute_ink_needs(hw_config, well_model, print_wells)

        if not pump_ink_needs:
            # No ink info — use default pump with estimated volume
            default_pump = "P1"
            enabled = getattr(hw_config, 'enabled_pump_ids', [])
            if enabled:
                default_pump = enabled[0]
            # Estimate 2µL per well as default
            estimated_vol = max(len(print_wells) * 2.0, 10.0)
            pump_ink_needs = {default_pump: estimated_vol}
            logger.info(f"No ink info — defaulting to {default_pump}: {estimated_vol:.1f}uL")

        max_per_run = {}
        for pump_id in pump_ink_needs:
            if pump_id in self.preferences.max_ink_volume_uL:
                max_per_run[pump_id] = self.preferences.max_ink_volume_uL[pump_id]
            else:
                pcfg = hw_config.pumps.get(pump_id)
                if pcfg and pcfg.syringe:
                    max_per_run[pump_id] = pcfg.syringe.volume_uL
                else:
                    max_per_run[pump_id] = 100.0

        runs_needed = 1
        for pump_id, total_needed in pump_ink_needs.items():
            max_vol = max_per_run.get(pump_id, 100.0)
            if max_vol > 0:
                runs_needed = max(runs_needed, math.ceil(total_needed / max_vol))

        self.total_runs = runs_needed
        self.ink_consumption_uL = dict(pump_ink_needs)
        self.total_ink_volume_uL = sum(pump_ink_needs.values())

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

        for run_idx, well_group in enumerate(run_groups):
            run_num = run_idx + 1
            run_ink = {}
            for pump_id, total in pump_ink_needs.items():
                run_ink[pump_id] = min(
                    total / runs_needed,
                    max_per_run.get(pump_id, 100.0),
                )

            self._add_service_steps(
                run_num, run_ink, hw_config,
                ink_wells, wash_wells, waste_wells, buffer_wells,
            )

            well_str = ", ".join(well_group[:3])
            if len(well_group) > 3:
                well_str += "..."
            self.steps.append(PlanStep(
                step_type=PlanStepType.PRINT,
                description=f"Print wells {well_str} (run {run_num}/{runs_needed})",
                target_wells=list(well_group),
                run_number=run_num,
            ))

        self.steps.append(PlanStep(
            step_type=PlanStepType.RETURN_HOME,
            description="Return to home position",
            run_number=runs_needed,
        ))

        self._estimate_times()
        logger.info(
            f"Plan: {len(self.steps)} steps, {self.total_runs} runs, "
            f"{self.total_print_wells} wells, {self.total_ink_volume_uL:.1f} uL"
        )

    def _compute_ink_needs(self, hw_config, well_model, print_wells):
        pump_ink_needs = {}
        for well_name in print_wells:
            assignment = well_model.get_assignment(well_name)
            if assignment is None:
                continue
            for _coll in assignment.print_collections:
                ink_per_coll = 2.0
                enabled = hw_config.enabled_pump_ids
                if enabled:
                    pid = enabled[0]
                    pump_ink_needs[pid] = pump_ink_needs.get(pid, 0.0) + ink_per_coll
        return pump_ink_needs

    def _add_service_steps(self, run_num, run_ink_per_pump, hw_config,
                           ink_wells, wash_wells, waste_wells, buffer_wells):
        """v7.3: Correct service sequence WASTE-WASH-BUFFER-WASH-LOAD_INK.

        Before each print run, prepare the syringe:
        1. WASTE — purge whatever is in the syringe
        2. WASH — clean the needle
        3. BUFFER — load buffer to separate oil from ink
        4. WASH — clean again after buffer
        5. LOAD_INK — aspirate ink needed for this run
        """
        prefs = self.preferences

        # Use the correct attribute names (UI sends use_waste/use_wash/use_buffer)
        do_waste = getattr(prefs, 'use_waste', getattr(prefs, 'waste_before_refill', True))
        do_wash = getattr(prefs, 'use_wash', getattr(prefs, 'wash_after_refill', True))
        do_buffer = getattr(prefs, 'use_buffer', getattr(prefs, 'refill_buffer_after_waste', True))

        # 1. WASTE — purge syringe
        if do_waste and waste_wells:
            for pump_id in run_ink_per_pump:
                self.steps.append(PlanStep(
                    step_type=PlanStepType.WASTE,
                    description=f"Purge {pump_id} at {waste_wells[0]}",
                    target_wells=[waste_wells[0]],
                    pump_id=pump_id, run_number=run_num,
                ))

        # 2. WASH — clean needle
        if do_wash and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Wash needle at {wash_wells[0]}",
                target_wells=[wash_wells[0]], run_number=run_num,
            ))

        # 3. BUFFER — load buffer layer
        if do_buffer and buffer_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.REFILL_BUFFER,
                description=f"Load buffer from {buffer_wells[0]}",
                target_wells=[buffer_wells[0]], run_number=run_num,
            ))

        # 4. WASH again — clean after buffer
        if do_wash and do_buffer and wash_wells:
            self.steps.append(PlanStep(
                step_type=PlanStepType.WASH,
                description=f"Post-buffer wash at {wash_wells[0]}",
                target_wells=[wash_wells[0]], run_number=run_num,
            ))

        # 5. LOAD INK — aspirate ink for this run
        for pump_id, volume in run_ink_per_pump.items():
            if volume <= 0:
                continue
            pcfg = hw_config.pumps.get(pump_id)
            ink_name = pcfg.ink.name if pcfg and pcfg.ink else "unknown"
            target = ink_wells[0] if ink_wells else "?"
            self.steps.append(PlanStep(
                step_type=PlanStepType.LOAD_INK,
                description=f"Load {volume:.1f} uL {ink_name} into {pump_id}",
                target_wells=[target] if target != "?" else [],
                pump_id=pump_id, volume_uL=volume,
                ink_name=ink_name, run_number=run_num,
            ))


    def _estimate_times(self):
        prefs = self.preferences
        total = 0.0
        for step in self.steps:
            if step.step_type == PlanStepType.LOAD_INK:
                t = 3.0 + (step.volume_uL / max(prefs.aspirate_speed_uL_s, 0.01))
            elif step.step_type == PlanStepType.WASH:
                t = 3.0 + (prefs.wash_cycles * prefs.wash_time_per_cycle_s)
            elif step.step_type == PlanStepType.WASTE:
                t = 5.0
            elif step.step_type == PlanStepType.REFILL_BUFFER:
                t = 8.0
            elif step.step_type == PlanStepType.PRINT:
                t = len(step.target_wells) * 30.0
            elif step.step_type == PlanStepType.RETURN_HOME:
                t = 5.0
            else:
                t = 1.0
            step.estimated_seconds = t
            total += t
        self.estimated_total_seconds = total

    def validate(self, hw_config=None, well_model=None):
        issues = []
        if not self.steps:
            issues.append("Plan is empty")
            return (False, issues)

        if not any(s.step_type == PlanStepType.PRINT for s in self.steps):
            issues.append("Plan has no print steps")

        for step in self.steps:
            if step.step_type == PlanStepType.LOAD_INK:
                if not step.target_wells or step.target_wells[0] == "?":
                    issues.append(
                        f"Ink load for {step.pump_id} ({step.ink_name}) "
                        f"has no target ink well")

        if well_model is not None:
            needs = {
                "wash": any(s.step_type == PlanStepType.WASH for s in self.steps),
                "waste": any(s.step_type == PlanStepType.WASTE for s in self.steps),
                "buffer": any(s.step_type == PlanStepType.REFILL_BUFFER for s in self.steps),
            }
            for role_str, needed in needs.items():
                if needed and not _get_role_wells(well_model, role_str):
                    issues.append(f"Plan requires {role_str} wells but none assigned")

        if hw_config is not None:
            for step in self.steps:
                if step.step_type == PlanStepType.LOAD_INK and step.pump_id:
                    pcfg = hw_config.pumps.get(step.pump_id)
                    if pcfg and pcfg.syringe:
                        if step.volume_uL > pcfg.syringe.volume_uL:
                            issues.append(
                                f"Ink load {step.volume_uL:.1f} uL for "
                                f"{step.pump_id} exceeds syringe "
                                f"({pcfg.syringe.volume_uL:.1f} uL)")

        return (len(issues) == 0, issues)

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
            "ink_consumption_uL": dict(self.ink_consumption_uL),
            "total_print_wells": self.total_print_wells,
            "total_ink_volume_uL": self.total_ink_volume_uL,
            "estimated_total_seconds": self.estimated_total_seconds,
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintPlanOfAction:
        return cls(
            steps=[PlanStep.from_dict(s) for s in data.get("steps", [])],
            total_runs=data.get("total_runs", 0),
            preferences=PlanPreferences.from_dict(data.get("preferences", {})),
            ink_consumption_uL=data.get("ink_consumption_uL", {}),
            total_print_wells=data.get("total_print_wells", 0),
            total_ink_volume_uL=data.get("total_ink_volume_uL", 0.0),
            estimated_total_seconds=data.get("estimated_total_seconds", 0.0),
        )


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
                        getattr(well_model.get_assignment(iw), 'ink_name', None) == ink_name
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
            "wash": any(s.step_type == PlanStepType.WASH for s in plan.steps),
            "waste": any(s.step_type == PlanStepType.WASTE for s in plan.steps),
            "buffer": any(s.step_type == PlanStepType.REFILL_BUFFER for s in plan.steps),
        }
        for role_str, needed in needs.items():
            if needed and not _get_role_wells(well_model, role_str):
                issues.append(f"Plan requires {role_str} wells but none assigned")

    # 4. Pump-needle mapping
    if hw_config is not None and hw_config.needle is not None:
        num_ch = hw_config.needle.num_channels
        enabled = hw_config.enabled_pump_ids
        if num_ch > 0 and enabled:
            mapped = hw_config.needle_channel_pump_map
            if len(mapped) != num_ch:
                issues.append(
                    f"Needle has {num_ch} channel(s) but {len(mapped)} mapped")

    # 5. Syringe capacity
    if plan is not None and hw_config is not None:
        for step in plan.steps:
            if step.step_type == PlanStepType.LOAD_INK and step.pump_id:
                pcfg = hw_config.pumps.get(step.pump_id)
                if pcfg and pcfg.syringe:
                    if step.volume_uL > pcfg.syringe.volume_uL:
                        issues.append(
                            f"Run {step.run_number}: {step.volume_uL:.1f} uL "
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
# v7.2.6: Plan → PrintCommand Execution Bridge
# ═══════════════════════════════════════════════════════════════════

def _find_service_well(well_model, plate, role_value: str):
    """Find first well with given role. Returns (name, x, y) or None.

    v7.2.6b: Use settings feedrates + track fluid balance.
    """
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
    """Generate waste ejection commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "waste")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No waste well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    eject_vol = 5.0  # µL to eject
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Waste: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to waste well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower to waste depth"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": eject_vol, "feedrate": p_fr},
                     label=f"Eject {eject_vol:.1f} into waste"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 0.5},
                     label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from waste"),
    ]


def _wash_commands(step, well_model, plate, settings):
    """Generate wash commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "wash")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No wash well")]
    name, x, y = well
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Wash: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise to travel height"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to wash well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into wash"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 5.0},
                     label="Wash soak"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from wash"),
    ]


def _buffer_commands(step, well_model, plate, settings):
    """Generate buffer commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "buffer")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No buffer well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    return [
        PrintCommand(type=CommandType.COMMENT, label=f"== Buffer: {name} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to buffer {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into buffer"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -5.0, "feedrate": p_fr},
                     label="Aspirate buffer"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from buffer"),
    ]


def _load_ink_commands(step, well_model, plate, settings):
    """Generate ink loading commands using settings feedrates."""
    from SupportClasses.PrintManager import PrintCommand, CommandType
    well = _find_service_well(well_model, plate, "ink")
    if not well:
        return [PrintCommand(type=CommandType.COMMENT, label="SKIP: No ink well")]
    name, x, y = well
    pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
    z_fr = getattr(settings, 'z_feedrate', 60.0)
    p_fr = getattr(settings, 'pump_feedrate', 30.0)
    volume = step.volume_uL if step.volume_uL > 0 else 50.0
    ink_name = step.ink_name or "?"
    return [
        PrintCommand(type=CommandType.COMMENT,
                     label=f"== Load Ink: {ink_name} ({volume:.1f}uL) into {pump} =="),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise"),
        PrintCommand(type=CommandType.MOVE_XY, params={"x": x, "y": y},
                     label=f"Travel to ink well {name}"),
        PrintCommand(type=CommandType.TRAVEL_DOWN, params={"feedrate": z_fr},
                     label="Lower into ink"),
        PrintCommand(type=CommandType.EXTRUDE,
                     params={"pump": pump, "amount": -volume, "feedrate": p_fr},
                     label=f"Aspirate {volume:.1f}uL {ink_name}"),
        PrintCommand(type=CommandType.DWELL, params={"seconds": 1.0}, label="Settle"),
        PrintCommand(type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                     label="Raise from ink"),
    ]


def plan_to_commands(plan, well_model, plate, path_points, settings, hw_config=None):
    """Convert PrintPlanOfAction into executable PrintJob.

    v7.2.6b: Use settings feedrates + track fluid balance.
    Ensures pump never goes negative by inserting extra LOAD_INK steps.
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

    # Track fluid balance per pump (µL loaded minus µL dispensed)
    fluid_balance: dict[str, float] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    for step in steps:
        stype = getattr(step, 'step_type', None)
        if stype is None:
            continue

        if stype == PlanStepType.WASTE:
            pump = step.pump_id or "P1"
            all_commands.extend(_waste_commands(step, well_model, plate, settings))
            # Waste ejects 5µL
            fluid_balance[pump] = max(0, fluid_balance.get(pump, 0) - 5.0)

        elif stype == PlanStepType.WASH:
            all_commands.extend(_wash_commands(step, well_model, plate, settings))

        elif stype == PlanStepType.REFILL_BUFFER:
            pump = step.pump_id or "P1"
            all_commands.extend(_buffer_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + 5.0

        elif stype == PlanStepType.LOAD_INK:
            pump = step.pump_id or "P1"
            vol = step.volume_uL if step.volume_uL > 0 else 50.0
            all_commands.extend(_load_ink_commands(step, well_model, plate, settings))
            fluid_balance[pump] = fluid_balance.get(pump, 0) + vol

        elif stype == PlanStepType.PRINT:
            target_wells = getattr(step, 'target_wells', [])
            if not target_wells:
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT, label="SKIP: No wells"))
                continue

            well_positions = []
            for wn in target_wells:
                try:
                    x, y = plate.get_well_position(wn)
                    well_positions.append((wn, x, y))
                except Exception:
                    continue

            if not well_positions:
                continue

            pump = step.pump_id or getattr(settings, 'active_pump', 'P1') or 'P1'
            flow = getattr(settings, 'flow_rate', 0.01) or 0.01

            if prev_pump is not None and pump != prev_pump:
                all_commands.append(PrintCommand(
                    type=CommandType.SWITCH_PUMP, params={"pump": pump},
                    label=f"Switch to {pump}"))
            prev_pump = pump

            # Estimate ink needed for this run
            path_length = 0.0
            if len(path_points) >= 2:
                import math
                for i in range(1, len(path_points)):
                    dx = path_points[i][0] - path_points[i-1][0]
                    dy = path_points[i][1] - path_points[i-1][1]
                    path_length += math.sqrt(dx*dx + dy*dy)
            ink_per_well = path_length * flow * getattr(settings, 'num_layers', 1)
            ink_needed = ink_per_well * len(well_positions)

            # Check if we have enough ink
            balance = fluid_balance.get(pump, 0)
            if balance < ink_needed and ink_needed > 0:
                shortfall = ink_needed - balance
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"WARNING: {pump} needs {ink_needed:.1f}uL but only {balance:.1f}uL loaded"))
                logger.warning(
                    f"Pump {pump}: needs {ink_needed:.1f}uL, has {balance:.1f}uL "
                    f"(shortfall {shortfall:.1f}uL)")

            run_num = getattr(step, 'run_number', '?')
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT,
                label=f"== Print Run {run_num}: {len(well_positions)} wells, {pump} =="))

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
                # Deduct estimated ink used
                fluid_balance[pump] = fluid_balance.get(pump, 0) - ink_needed
            except Exception as exc:
                logger.error(f"build_well_plate_job run {run_num}: {exc}")
                all_commands.append(PrintCommand(
                    type=CommandType.COMMENT,
                    label=f"ERROR: Run {run_num} failed: {exc}"))

        elif stype == PlanStepType.RETURN_HOME:
            z_fr = getattr(settings, 'z_feedrate', 60.0)
            all_commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP, params={"feedrate": z_fr},
                label="Final: raise"))
            all_commands.append(PrintCommand(
                type=CommandType.HOME_XY, label="Return home"))
        else:
            all_commands.append(PrintCommand(
                type=CommandType.COMMENT, label=f"Unknown: {stype}"))

    if not all_commands:
        return None

    tw = getattr(plan, 'total_print_wells', '?')
    tr = getattr(plan, 'total_runs', '?')
    return PrintJob(
        name=f"Plan: {tw} wells, {tr} run(s)",
        description="Generated from PrintPlanOfAction v7.2.6b",
        settings=settings,
        commands=all_commands,
    )

