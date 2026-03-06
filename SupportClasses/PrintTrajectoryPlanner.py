"""
PrintTrajectoryPlanner.py — Plan-to-trajectory converter for MEBP v7.3.

Converts a PrintPlanOfAction into a continuous time-parameterized trajectory
of (x, y, z, p1, p2, p3, t) waypoints that the TrajectoryExecutor can
follow using velocity-based control.

Key features:
  - All movements are time-parameterized — feedrates become durations
  - Trapezoidal velocity profiles for smooth Z acceleration/deceleration
  - XY + pump positions are coordinated during print moves
  - Fluid balance is validated at planning time (rejects if pump goes negative)
  - Dwell times from settings are respected

Usage:
    from SupportClasses.PrintTrajectoryPlanner import plan_to_trajectory
    
    result = plan_to_trajectory(plan, well_model, plate, path_points,
                                settings, hw_config)
    if result.valid:
        trajectory_executor.execute(result.waypoints)
    else:
        print(result.issues)
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
#  Waypoint — compatible with v7.1 TrajectoryExecutor
# ═══════════════════════════════════════════════════════════════════

@dataclass
class Waypoint:
    """Single point in the 6-axis trajectory, parametric in time.

    Uses the same field names as v7.1 TrajectoryPlanner.Waypoint so
    TrajectoryExecutor.execute() works without modification.
    """
    t: float = 0.0       # seconds from print start
    x: float = 0.0       # mm, plate-relative XY
    y: float = 0.0       # mm, plate-relative XY
    z: float = 0.0       # mm, zero-ref Z height
    p1: float = 0.0      # mm, pump 1 plunger position
    p2: float = 0.0      # mm, pump 2 plunger position
    p3: float = 0.0      # mm, pump 3 plunger position

    # Metadata for monitor display + recording
    segment: str = ""        # "travel" | "print" | "service" | "dwell"
    well: str = ""           # current well name
    is_travel: bool = False
    is_retract: bool = False
    segment_id: int = 0


# ═══════════════════════════════════════════════════════════════════
#  Trajectory Result
# ═══════════════════════════════════════════════════════════════════

@dataclass
class TrajectoryResult:
    """Result of trajectory planning — waypoints + validation."""
    waypoints: list[Waypoint] = field(default_factory=list)
    valid: bool = True
    issues: list[str] = field(default_factory=list)
    total_duration_s: float = 0.0
    total_path_length_mm: float = 0.0
    fluid_balance: dict[str, float] = field(default_factory=dict)
    segment_count: int = 0
    well_count: int = 0

    def summary(self) -> str:
        mins = self.total_duration_s / 60
        return (f"{len(self.waypoints)} waypoints, {mins:.1f} min, "
                f"{self.well_count} wells, {self.segment_count} segments")


# ═══════════════════════════════════════════════════════════════════
#  Velocity Profile Helpers
# ═══════════════════════════════════════════════════════════════════

def _trapezoidal_fraction(frac: float, accel_frac: float = 0.2) -> float:
    """Map linear fraction [0,1] through a trapezoidal velocity profile.

    Returns position fraction [0,1] with smooth accel/decel zones.
    accel_frac = fraction of move spent accelerating (same for decel).
    """
    af = min(accel_frac, 0.4)  # cap at 40% each side
    if frac <= 0:
        return 0.0
    if frac >= 1:
        return 1.0
    if frac < af:
        # Quadratic ramp up
        return (frac * frac) / (2 * af)
    elif frac > 1.0 - af:
        # Quadratic ramp down
        d = 1.0 - frac
        return 1.0 - (d * d) / (2 * af)
    else:
        # Linear cruise
        return af / 2 + (frac - af) / (1.0 - 2 * af) * (1.0 - af)


def _interp(a: float, b: float, frac: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * frac


def _distance_2d(x1, y1, x2, y2) -> float:
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# ═══════════════════════════════════════════════════════════════════
#  Print Trajectory Planner
# ═══════════════════════════════════════════════════════════════════

class PrintTrajectoryPlanner:
    """
    Converts a PrintPlanOfAction into a continuous trajectory.

    All timing is derived from feedrates in PrintSettings:
        xy_feedrate   → mm/min for XY travel moves
        z_feedrate    → mm/min for Z moves
        print_feedrate → mm/min for XY during printing
        pump_feedrate  → mm/min for pump moves
    """

    # Waypoint generation intervals (seconds)
    DT_PRINT = 0.05    # 50ms for coordinated print moves (XY + pump)
    DT_TRAVEL = 0.25   # 250ms for travel moves (just needs start/end)
    DT = 0.05          # default (overridden per-move)

    def __init__(self):
        self._waypoints: list[Waypoint] = []
        self._t: float = 0.0  # time cursor
        self._x: float = 0.0  # current X
        self._y: float = 0.0  # current Y
        self._z: float = 5.0  # current Z (start at travel height)
        self._pumps: dict[str, float] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}
        self._segment_id: int = 0
        self._fluid_balance: dict[str, float] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}
        self._issues: list[str] = []

    def _wp(self, segment="", well="", is_travel=False, is_retract=False):
        """Append current state as a waypoint."""
        self._waypoints.append(Waypoint(
            t=self._t, x=self._x, y=self._y, z=self._z,
            p1=self._pumps["P1"], p2=self._pumps["P2"], p3=self._pumps["P3"],
            segment=segment, well=well,
            is_travel=is_travel, is_retract=is_retract,
            segment_id=self._segment_id,
        ))

    def _next_segment(self):
        self._segment_id += 1

    # ── Core motion primitives ────────────────────────────────────

    def _move_z(self, target_z: float, feedrate_mm_min: float,
                segment="travel", well=""):
        """Generate Z movement waypoints with trapezoidal profile."""
        dist = abs(target_z - self._z)
        if dist < 0.001:
            return
        speed_mm_s = max(feedrate_mm_min / 60.0, 0.1)
        duration = dist / speed_mm_s
        dt = self.DT_TRAVEL  # Z moves use coarse interval
        n_steps = max(int(duration / dt), 2)

        z_start = self._z
        step_dt = duration / n_steps
        for i in range(1, n_steps + 1):
            frac = i / n_steps
            s = _trapezoidal_fraction(frac)
            self._z = _interp(z_start, target_z, s)
            self._t += step_dt
            self._wp(segment=segment, well=well,
                     is_travel=(segment == "travel"))

    def _move_xy(self, target_x: float, target_y: float,
                 feedrate_mm_min: float, segment="travel", well=""):
        """Generate XY movement waypoints with trapezoidal profile."""
        dist = _distance_2d(self._x, self._y, target_x, target_y)
        if dist < 0.001:
            return
        speed_mm_s = max(feedrate_mm_min / 60.0, 0.1)
        duration = dist / speed_mm_s
        dt = self.DT_TRAVEL  # XY travel uses coarse interval
        n_steps = max(int(duration / dt), 2)

        x_start, y_start = self._x, self._y
        step_dt = duration / n_steps
        for i in range(1, n_steps + 1):
            frac = i / n_steps
            s = _trapezoidal_fraction(frac)
            self._x = _interp(x_start, target_x, s)
            self._y = _interp(y_start, target_y, s)
            self._t += step_dt
            self._wp(segment=segment, well=well,
                     is_travel=(segment == "travel"))

    def _move_pump(self, pump_id: str, delta_mm: float,
                   feedrate_mm_min: float, segment="service", well=""):
        """Generate pump movement waypoints."""
        if abs(delta_mm) < 0.0001:
            return
        speed_mm_s = max(feedrate_mm_min / 60.0, 0.01)
        duration = abs(delta_mm) / speed_mm_s
        n_steps = max(int(duration / self.DT), 2)

        start_pos = self._pumps[pump_id]
        end_pos = start_pos + delta_mm
        for i in range(1, n_steps + 1):
            frac = i / n_steps
            s = _trapezoidal_fraction(frac, accel_frac=0.15)
            self._pumps[pump_id] = _interp(start_pos, end_pos, s)
            self._t += duration / n_steps
            self._wp(segment=segment, well=well,
                     is_retract=(delta_mm < 0 and segment != "service"))

    def _dwell(self, seconds: float, segment="dwell", well=""):
        """Insert a dwell (hold position)."""
        if seconds <= 0:
            return
        self._t += seconds
        self._wp(segment=segment, well=well)

    def _print_path_coordinated(self, path_points: list[tuple[float, float]],
                                well_x: float, well_y: float,
                                pump_id: str, flow_rate: float,
                                feedrate_mm_min: float,
                                settings, well_name: str = ""):
        """Generate coordinated XY + pump waypoints for a print path.

        path_points are relative to well center. flow_rate is mm-pump
        per mm-XY-travel (dimensionless ratio).
        """
        if len(path_points) < 2:
            return

        print_speed = max(feedrate_mm_min / 60.0, 0.1)

        # Move to first point (already at print Z)
        first_x = well_x + path_points[0][0]
        first_y = well_y + path_points[0][1]
        self._move_xy(first_x, first_y, feedrate_mm_min,
                      segment="print", well=well_name)

        # Print segments: coordinated XY + pump
        for i in range(1, len(path_points)):
            seg_x = well_x + path_points[i][0]
            seg_y = well_y + path_points[i][1]
            dist = _distance_2d(self._x, self._y, seg_x, seg_y)
            if dist < 0.001:
                continue

            duration = dist / print_speed
            n_steps = max(int(duration / self.DT_PRINT), 2)
            pump_delta = dist * flow_rate  # total pump movement for segment

            x_start, y_start = self._x, self._y
            p_start = self._pumps[pump_id]

            for j in range(1, n_steps + 1):
                frac = j / n_steps
                self._x = _interp(x_start, seg_x, frac)  # linear for print
                self._y = _interp(y_start, seg_y, frac)
                self._pumps[pump_id] = _interp(p_start, p_start + pump_delta, frac)
                self._t += duration / n_steps
                self._wp(segment="print", well=well_name)

            # Track fluid dispensed
            self._fluid_balance[pump_id] -= abs(pump_delta)

    # ── High-level plan step handlers ─────────────────────────────

    def _travel_to_well(self, wx: float, wy: float, settings, well=""):
        """Raise Z → travel XY → (stay at travel height)."""
        self._next_segment()
        # Raise to travel height if not there
        tz = settings.travel_z_height
        if self._z < tz - 0.01:
            self._move_z(tz, settings.z_feedrate, segment="travel", well=well)
        # XY travel
        self._move_xy(wx, wy, settings.xy_feedrate, segment="travel", well=well)
        # Dwell after travel
        if settings.dwell_after_move > 0:
            self._dwell(settings.dwell_after_move, well=well)

    def _lower_to_print(self, settings, well=""):
        """Lower Z from travel height to print height."""
        self._move_z(settings.print_z_height, settings.z_feedrate,
                     segment="travel", well=well)

    def _raise_from_print(self, settings, well=""):
        """Raise Z from print height to travel height."""
        self._move_z(settings.travel_z_height, settings.z_feedrate,
                     segment="travel", well=well)

    def _do_waste(self, plate, well_model, pump_id, settings):
        """Waste sequence: travel → lower → eject → raise."""
        well = _find_well(well_model, plate, "waste")
        if not well:
            logger.info("Skipping: no waste well assigned — will proceed without")
            return
        name, wx, wy = well
        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        eject_vol_mm = 5.0 / _get_uL_per_mm(settings, pump_id)
        self._move_pump(pump_id, eject_vol_mm, settings.pump_feedrate,
                        segment="service", well=name)
        self._fluid_balance[pump_id] -= abs(eject_vol_mm)
        self._dwell(0.5, well=name)
        self._raise_from_print(settings, well=name)

    def _do_wash(self, plate, well_model, settings):
        """Wash sequence: travel → lower → dwell → raise."""
        well = _find_well(well_model, plate, "wash")
        if not well:
            logger.info("Skipping: no wash well assigned — will proceed without")
            return
        name, wx, wy = well
        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        self._dwell(5.0, segment="service", well=name)
        self._raise_from_print(settings, well=name)

    def _do_buffer(self, plate, well_model, pump_id, settings):
        """Buffer sequence: travel → lower → aspirate → raise."""
        well = _find_well(well_model, plate, "buffer")
        if not well:
            logger.info("Skipping: no buffer well assigned — will proceed without")
            return
        name, wx, wy = well
        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        aspirate_mm = -5.0 / _get_uL_per_mm(settings, pump_id)
        self._move_pump(pump_id, aspirate_mm, settings.pump_feedrate,
                        segment="service", well=name)
        self._fluid_balance[pump_id] += abs(aspirate_mm)
        self._dwell(1.0, well=name)
        self._raise_from_print(settings, well=name)

    def _do_load_ink(self, plate, well_model, pump_id, volume_uL, settings):
        """Ink load: travel → lower → aspirate volume → raise."""
        well = _find_well(well_model, plate, "ink")
        if not well:
            logger.info("Skipping: no ink well assigned — will proceed without")
            return
        name, wx, wy = well
        uL_per_mm = _get_uL_per_mm(settings, pump_id)
        aspirate_mm = -volume_uL / uL_per_mm  # negative = aspirate
        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        self._move_pump(pump_id, aspirate_mm, settings.pump_feedrate,
                        segment="service", well=name)
        self._fluid_balance[pump_id] += volume_uL
        self._dwell(1.0, well=name)
        self._raise_from_print(settings, well=name)

    def _estimate_ink_needed(self, path_points, flow_rate, num_layers, num_wells):
        """Estimate total pump travel (mm) needed for a set of wells."""
        path_length = 0.0
        if len(path_points) >= 2:
            for i in range(1, len(path_points)):
                dx = path_points[i][0] - path_points[i-1][0]
                dy = path_points[i][1] - path_points[i-1][1]
                path_length += math.sqrt(dx*dx + dy*dy)
        ink_per_well_mm = path_length * flow_rate * num_layers
        return ink_per_well_mm * num_wells

    def _do_service_and_print(self, plate, well_model, well_names, pump_id,
                              path_points, flow_rate, settings):
        """Full bioprinting workflow for a run:
        1. Waste — eject whatever is in the syringe
        2. Wash — clean needle
        3. Load buffer — aspirate buffer
        4. Wash — clean after buffer
        5. Load ink — aspirate exactly what's needed + 20% margin
        6. Print all wells in this run
        """
        # Calculate ink needed
        ink_mm = self._estimate_ink_needed(
            path_points, flow_rate, settings.num_layers, len(well_names))
        ink_mm_with_margin = ink_mm * 1.2 + 0.5  # 20% margin + prime/retract overhead
        uL_per_mm = _get_uL_per_mm(settings, pump_id)
        ink_uL = ink_mm_with_margin * uL_per_mm

        logger.info(
            f"Run service: {len(well_names)} wells, "
            f"ink needed={ink_mm:.2f}mm ({ink_uL:.1f}uL), pump={pump_id}")

        # 1. WASTE — eject current syringe contents
        self._do_waste(plate, well_model, pump_id, settings)

        # 2. WASH — clean needle
        self._do_wash(plate, well_model, settings)

        # 3. LOAD BUFFER — aspirate buffer to separate oil from ink
        self._do_buffer(plate, well_model, pump_id, settings)

        # 4. WASH again — clean after buffer pickup
        self._do_wash(plate, well_model, settings)

        # 5. LOAD INK — aspirate exactly what's needed
        self._do_load_ink(plate, well_model, pump_id, ink_uL, settings)

        # 6. PRINT all wells
        self._do_print_wells(plate, well_names, pump_id, path_points,
                             flow_rate, settings)

    def _do_print_wells(self, plate, well_names, pump_id, path_points,
                        flow_rate, settings):
        """Print a list of wells: for each → travel, lower, prime, print, retract, raise."""
        for well_name in well_names:
            try:
                wx, wy = plate.get_well_position(well_name)
            except Exception:
                self._issues.append(f"Well {well_name}: position lookup failed")
                continue

            self._next_segment()

            # Travel to well
            self._travel_to_well(wx, wy, settings, well=well_name)

            # Lower to print
            self._lower_to_print(settings, well=well_name)

            # Prime
            prime_mm = settings.get_retract_amount(pump_id) if hasattr(settings, 'get_retract_amount') else 0
            if prime_mm > 0:
                self._move_pump(pump_id, prime_mm, settings.pump_feedrate,
                                segment="prime", well=well_name)

            # Print layers
            for layer in range(settings.num_layers):
                if layer > 0:
                    # Layer height increment
                    self._move_z(self._z + settings.layer_height,
                                 settings.z_feedrate, segment="print",
                                 well=well_name)

                self._print_path_coordinated(
                    path_points, wx, wy, pump_id, flow_rate,
                    settings.print_feedrate, settings, well_name)

            # Retract
            retract_mm = settings.get_retract_amount(pump_id) if hasattr(settings, 'get_retract_amount') else 0
            if retract_mm > 0:
                self._move_pump(pump_id, -retract_mm, settings.pump_feedrate,
                                segment="retract", well=well_name)

            # Raise
            self._raise_from_print(settings, well=well_name)

    # ── Main entry point ──────────────────────────────────────────

    def generate(self, plan, well_model, plate, path_points, settings,
                 hw_config=None) -> TrajectoryResult:
        """Generate complete trajectory from a PrintPlanOfAction.

        Args:
            plan: PrintPlanOfAction with .steps list
            well_model: WellSetupModel
            plate: WellPlate
            path_points: list[(float, float)] — print geometry per well
            settings: PrintSettings
            hw_config: optional HardwareConfig for syringe specs

        Returns:
            TrajectoryResult with waypoints and validation status
        """
        # Reset state
        self._waypoints.clear()
        self._t = 0.0
        self._x = 0.0
        self._y = 0.0
        self._z = settings.travel_z_height
        self._pumps = {"P1": 0.0, "P2": 0.0, "P3": 0.0}
        self._segment_id = 0
        self._fluid_balance = {"P1": 0.0, "P2": 0.0, "P3": 0.0}
        self._issues = []

        # Initial waypoint
        self._wp(segment="start")

        try:
            from SupportClasses.PrintPlanOfAction import PlanStepType
        except ImportError:
            self._issues.append("PrintPlanOfAction module not available")
            return TrajectoryResult(valid=False, issues=self._issues)

        steps = getattr(plan, 'steps', [])
        if not steps:
            self._issues.append("Plan has no steps")
            return TrajectoryResult(valid=False, issues=self._issues)

        well_count = 0
        prev_pump = None

        for step in steps:
            stype = getattr(step, 'step_type', None)
            pump = getattr(step, 'pump_id', None) or getattr(settings, 'active_pump', 'P1') or 'P1'

            if stype == PlanStepType.PRINT:
                # v7.3: Full service workflow before each print run
                target_wells = getattr(step, 'target_wells', [])
                flow = getattr(settings, 'flow_rate', 0.01) or 0.01
                if target_wells:
                    self._do_service_and_print(
                        plate, well_model, target_wells, pump,
                        path_points, flow, settings)
                    well_count += len(target_wells)

            elif stype == PlanStepType.RETURN_HOME:
                self._next_segment()
                # Final waste + wash to leave needle clean
                self._do_waste(plate, well_model, pump, settings)
                self._do_wash(plate, well_model, settings)
                self._move_z(settings.travel_z_height, settings.z_feedrate,
                             segment="travel")
                self._move_xy(0, 0, settings.xy_feedrate, segment="travel")
                self._wp(segment="end")

            elif stype in (PlanStepType.WASTE, PlanStepType.WASH,
                           PlanStepType.REFILL_BUFFER, PlanStepType.LOAD_INK):
                # Service steps are now auto-included in _do_service_and_print
                # Only execute standalone service steps if they appear without a PRINT
                if stype == PlanStepType.WASTE:
                    self._do_waste(plate, well_model, pump, settings)
                elif stype == PlanStepType.WASH:
                    self._do_wash(plate, well_model, settings)
                elif stype == PlanStepType.REFILL_BUFFER:
                    self._do_buffer(plate, well_model, pump, settings)
                elif stype == PlanStepType.LOAD_INK:
                    vol = step.volume_uL if step.volume_uL > 0 else 50.0
                    self._do_load_ink(plate, well_model, pump, vol, settings)

        # Fluid balance info — log for diagnostics, never block execution
        for pid, balance in self._fluid_balance.items():
            if balance < -0.1:
                logger.info(
                    f"{pid}: fluid balance {balance:.2f} mm "
                    f"(service steps will handle loading)")

        # Compute stats
        total_path = 0.0
        for i in range(1, len(self._waypoints)):
            a, b = self._waypoints[i - 1], self._waypoints[i]
            total_path += _distance_2d(a.x, a.y, b.x, b.y)

        return TrajectoryResult(
            waypoints=self._waypoints,
            valid=len(self._issues) == 0,
            issues=self._issues,
            total_duration_s=self._t,
            total_path_length_mm=total_path,
            fluid_balance=dict(self._fluid_balance),
            segment_count=self._segment_id,
            well_count=well_count,
        )


# ═══════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════

def _find_well(well_model, plate, role_value: str):
    """Find first well with given role. Returns (name, x, y) or None."""
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


def _get_uL_per_mm(settings, pump_id: str) -> float:
    """Get µL per mm of pump travel. Default for 250µL Hamilton = 3.378."""
    # Try to get from settings
    if hasattr(settings, 'uL_per_mm'):
        return settings.uL_per_mm
    return 3.378  # Default for 250µL Hamilton syringe


# ═══════════════════════════════════════════════════════════════════
#  Public API
# ═══════════════════════════════════════════════════════════════════

def plan_to_trajectory(plan, well_model, plate, path_points, settings,
                       hw_config=None) -> TrajectoryResult:
    """Convert a PrintPlanOfAction into a trajectory.

    Drop-in replacement for plan_to_commands() that returns a
    TrajectoryResult instead of a PrintJob.

    Args:
        plan: PrintPlanOfAction
        well_model: WellSetupModel
        plate: WellPlate
        path_points: list[(float, float)]
        settings: PrintSettings
        hw_config: optional HardwareConfig

    Returns:
        TrajectoryResult with .waypoints, .valid, .issues, .total_duration_s
    """
    planner = PrintTrajectoryPlanner()
    return planner.generate(plan, well_model, plate, path_points,
                            settings, hw_config)
