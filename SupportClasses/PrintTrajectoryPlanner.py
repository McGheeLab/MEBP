"""
PrintTrajectoryPlanner.py — Plan-to-trajectory converter for MEBP v7.3.

Converts a PrintPlanOfAction into a continuous time-parameterized trajectory
of (x, y, z, p1, p2, p3, t) waypoints that the TrajectoryExecutor can
follow using velocity-based control.

⚠️ v7.5.x KNOWN LIMITATION — Z POLARITY (see CLAUDE.md "Critical Safety Rules"):
This planner's Z geometry assumes the conventional convention that a LARGER Z is
physically HIGHER (``approach_z = top_z + 0.5``, ``if self._z < safe_z: raise``,
layer increments ``self._z + layer_height``, etc.). On machines where the needle
DESCENDS as raw Z increases (``StageController.ZDIR == -1`` — e.g. ME3B V1) these
gates invert: the Z-raise-before-travel may be skipped and "above the well" lands
BELOW it. The retract-before-cross-position-XY guarantee therefore does NOT hold
for the pure-trajectory print path on a ZDIR=-1 machine. Until this planner gets a
full polarity pass, prefer the discrete / hybrid-direct print path (DirectCommand
travel is polarity-agnostic). ``generate()`` logs a warning when ZDIR != 1.

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
        # v7.5.x: per-machine plate-local→stage axis sign, set from
        # settings.plate_axis_sign at the start of generate(). Applied to
        # GEOMETRIC well centres (plate.get_well_position) so they map to the
        # physically-correct well on a 180°-mounted stage (ME3B V1).
        self._plate_axis_sign: tuple = (1.0, 1.0)
        # v7.5.x: the active PrintSettings, captured at the start of generate()
        # / generate_inwell_print() so _well_xy can consult the CALIBRATED
        # well-position map (settings.well_positions_mm). None until set.
        self._settings = None

    def _well_xy(self, plate, well_name):
        """Well centre in ZERO-REF mm — prefers the CALIBRATED taught position
        (settings.well_positions_mm), else the GEOMETRIC plate-local offset
        (A1-relative mm) mapped onto the stage axes via the per-machine sign.
        See :func:`resolve_well_xy_mm`."""
        if self._settings is not None:
            return resolve_well_xy_mm(well_name, plate, self._settings)
        # Pre-generate fallback (settings not yet captured): legacy geometric.
        wx, wy = plate.get_well_position(well_name)
        sx, sy = self._plate_axis_sign
        return (sx * wx, sy * wy)

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

    def _print_path_coordinated(self, path_points, well_x, well_y,
                                pump_id, flow_rate, feedrate_mm_min,
                                settings, well_name=""):
        """Coordinated XY + pump waypoints. # v7.2.6-auto-B4

        Auto-derives pump flow rate from extrusion physics if
        settings.auto_pump_rate_uL_s > 0 (set by _compute_auto_settings).

        flow_rate here is pump-mm per XY-mm (dimensionless ratio).
        If auto_pump_rate_uL_s is set, it overrides the passed flow_rate.
        """
        if len(path_points) < 2:
            return

        print_speed = max(feedrate_mm_min / 60.0, 0.1)

        # Auto pump rate: derive flow_rate from physics if available
        auto_rate = getattr(settings, 'auto_pump_rate_uL_s', 0.0)
        if auto_rate > 0 and print_speed > 0:
            uL_per_mm_pump = _get_uL_per_mm(settings, pump_id)
            if uL_per_mm_pump > 0:
                # flow_rate (mm-pump / mm-XY) = (uL/s) / (mm/s * uL/mm)
                flow_rate = auto_rate / (print_speed * uL_per_mm_pump)

        # Move to first point
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
            pump_delta = dist * flow_rate

            x_start, y_start = self._x, self._y
            p_start = self._pumps[pump_id]

            for j in range(1, n_steps + 1):
                frac = j / n_steps
                self._x = _interp(x_start, seg_x, frac)
                self._y = _interp(y_start, seg_y, frac)
                self._pumps[pump_id] = _interp(p_start, p_start + pump_delta, frac)
                self._t += duration / n_steps
                self._wp(segment="print", well=well_name)

            self._fluid_balance[pump_id] -= abs(pump_delta)


    def _travel_to_well(self, wx: float, wy: float, settings, well="",
                        print_z: float | None = None):
        """4-phase smart Z approach. # v7.2.6-auto-B1

        Phase 1: Fast Z up to safe_z (travel_z_height) if not already there.
        Phase 2: Fast XY to destination at service_xy_speed_mm_s.
        Phase 3: Fast Z down to (top_z + 0.5mm buffer) — above well opening.
        Phase 4: Slow Z entry to print_z (per-well, v7.4.8) — controlled insertion.

        Falls back to single-phase if top_z_height == 0 (calibration not done).
        """
        self._next_segment()
        safe_z   = settings.travel_z_height
        top_z    = getattr(settings, 'top_z_height', 0.0)
        fast_z   = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z  = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        svc_xy   = getattr(settings, 'service_xy_speed_mm_s', 10.0) * 60.0  # → mm/min
        target   = print_z if print_z is not None else settings.print_z_height

        # Phase 1: raise to safe_z at fast speed
        if self._z < safe_z - 0.01:
            self._move_z(safe_z, fast_z, segment="travel", well=well)

        # Phase 2: fast XY travel
        self._move_xy(wx, wy, svc_xy, segment="travel", well=well)

        # Phase 3+4: lower into well if calibration data available
        if top_z > 0:
            approach_z = top_z + 0.5  # 0.5 mm buffer above well top
            # Phase 3: fast Z to just above well
            if self._z > approach_z + 0.01:
                self._move_z(approach_z, fast_z, segment="travel", well=well)
            # Phase 4: slow Z entry to print height
            self._move_z(target, entry_z, segment="travel", well=well)
        else:
            # Fallback: single-phase lower (no calibration)
            self._move_z(target, settings.z_feedrate,
                         segment="travel", well=well)

        # Dwell after arrival
        if settings.dwell_after_move > 0:
            self._dwell(settings.dwell_after_move, well=well)


    def _lower_to_print(self, settings, well="", print_z: float | None = None):
        """Lower Z to print height. 2-phase if calibration available. # v7.2.6-auto-B2

        Phase 1: Fast Z to (top_z + 0.5mm) if not already below that.
        Phase 2: Slow entry to print_z (per-well, v7.4.8).
        Falls back to single-phase (z_feedrate) if top_z_height == 0.
        """
        top_z   = getattr(settings, 'top_z_height', 0.0)
        fast_z  = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        target  = print_z if print_z is not None else settings.print_z_height

        if self._z <= target + 0.001:
            return  # Already at or below print height

        if top_z > 0:
            approach_z = top_z + 0.5
            if self._z > approach_z + 0.01:
                # Fast lower to just above well top
                self._move_z(approach_z, fast_z, segment="travel", well=well)
            # Slow entry into well
            self._move_z(target, entry_z, segment="travel", well=well)
        else:
            self._move_z(target, settings.z_feedrate, segment="travel", well=well)


    def _raise_from_print(self, settings, well=""):
        """Raise Z from print height to safe travel height. # v7.2.6-auto-B3

        Phase 1: Slow exit from well (entry_z_feedrate) to top_z + 0.5mm.
        Phase 2: Fast raise to safe_z (fast_z_feedrate) to clear obstacles.
        Falls back to single-phase if top_z_height == 0.
        """
        top_z   = getattr(settings, 'top_z_height', 0.0)
        fast_z  = getattr(settings, 'fast_z_feedrate_mm_min', settings.z_feedrate)
        entry_z = getattr(settings, 'entry_z_feedrate_mm_min', settings.z_feedrate)
        safe_z  = settings.travel_z_height

        if top_z > 0:
            clear_z = top_z + 0.5
            # Phase 1: slow exit from well
            if self._z < clear_z - 0.01:
                self._move_z(clear_z, entry_z, segment="travel", well=well)
            # Phase 2: fast raise to safe travel height
            if self._z < safe_z - 0.01:
                self._move_z(safe_z, fast_z, segment="travel", well=well)
        else:
            self._move_z(safe_z, settings.z_feedrate, segment="travel", well=well)


    def _do_waste(self, plate, well_model, pump_id, settings):
        """Waste: travel → lower → DISPENSE syringe contents → raise.
        # v7.2.6-dsf: waste clamp
        Dispenses only what is currently loaded (no overshoot).
        Uses service_pump_rate_uL_s from auto-settings if available.
        """
        well = _find_well(well_model, plate, "waste")
        if not well:
            logger.info("Skipping: no waste well assigned — will proceed without")
            return
        name, wx, wy = well
        wx, wy = self._well_xy(plate, name)  # map plate-local → stage axes
        uL_per_mm = _get_uL_per_mm(settings, pump_id)

        # Dispense the pump's current fluid balance (what was loaded)
        # Clamp to a safe maximum to avoid runaway
        fluid_loaded_mm = max(0.0, self._fluid_balance.get(pump_id, 0.0) / uL_per_mm
                              if uL_per_mm > 0 else 0.0)
        dispense_vol_mm = min(fluid_loaded_mm + (5.0 / uL_per_mm), 50.0 / uL_per_mm)
        dispense_vol_mm = max(dispense_vol_mm, 1.0 / uL_per_mm)  # at least 1 uL

        svc_fr = getattr(settings, 'pump_feedrate', 30.0)

        self._travel_to_well(wx, wy, settings, well=name)
        self._lower_to_print(settings, well=name)
        self._move_pump(pump_id, dispense_vol_mm, svc_fr,
                        segment="service", well=name)
        # After waste, pump balance resets to zero
        self._fluid_balance[pump_id] = 0.0
        self._dwell(0.5, well=name)
        self._raise_from_print(settings, well=name)


    def _do_wash(self, plate, well_model, settings):
        """Wash sequence: travel → lower → dwell → raise."""
        well = _find_well(well_model, plate, "wash")
        if not well:
            logger.info("Skipping: no wash well assigned — will proceed without")
            return
        name, wx, wy = well
        wx, wy = self._well_xy(plate, name)  # map plate-local → stage axes
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
        wx, wy = self._well_xy(plate, name)  # map plate-local → stage axes
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
        wx, wy = self._well_xy(plate, name)  # map plate-local → stage axes
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
        1. Waste — dispense whatever is in the syringe
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

        # 1. WASTE — dispense current syringe contents
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

    def _well_print_z(self, plate, well_name, settings) -> float:
        """v7.4.8: effective dispense Z for a well.

        If the well (e.g. a flattened rosette sub-well A1.a) prescribes an
        ``ink_z_mm`` (relative to the plate top), use
        ``top_z + z_up_sign * ink_z_mm``; otherwise fall back to the global
        ``settings.print_z_height``.

        v7.5.x: ``ink_z_mm`` keeps its "relative to plate top" meaning but is
        now applied along the reference-vector up-direction (``z_up_sign`` from
        ``StageController.print_z_dir()``), so a negative ``ink_z_mm`` means
        "into the well" on **both** Z polarities (the old ``top_z + ink_z_mm``
        inverted on ME3B V1, where larger Z is physically *lower*).
        """
        try:
            info = plate.get_well_info(well_name)
            if getattr(info, "ink_z_mm", None) is not None:
                top_z = getattr(settings, "top_z_height", 0.0)
                z_up = getattr(settings, "z_up_sign", 1.0)
                return top_z + z_up * info.ink_z_mm
        except Exception:
            pass
        return settings.print_z_height

    def _do_print_wells(self, plate, well_names, pump_id, path_points,
                        flow_rate, settings):
        """Print a list of wells: for each → travel, lower, prime, print, retract, raise."""
        for well_name in well_names:
            try:
                wx, wy = self._well_xy(plate, well_name)
            except Exception:
                self._issues.append(f"Well {well_name}: position lookup failed")
                continue

            self._next_segment()

            # v7.4.8: per-well dispense Z (rosette sub-wells / tubes).
            print_z = self._well_print_z(plate, well_name, settings)

            # Travel to well
            self._travel_to_well(wx, wy, settings, well=well_name,
                                 print_z=print_z)

            # Lower to print
            self._lower_to_print(settings, well=well_name, print_z=print_z)

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

    # ── In-well print trajectory (for hybrid execution) ───────────

    def generate_inwell_print(self, well_x, well_y, pump_id, path_points,
                              settings, well_name="") -> TrajectoryResult:
        """Generate trajectory for in-well printing ONLY.

        v7.2.9: Used by HybridPlanExecutor. Assumes the needle is already
        lowered to print_z_height inside the well. Generates ONLY:
          - Prime pump
          - Print path (coordinated XY + pump)
          - Retract pump

        No travel, no Z lower, no Z raise — DirectCommandExecutor handles
        all of that with blocking absolute moves.
        """
        self._waypoints = []
        self._t = 0.0
        self._x = well_x + (path_points[0][0] if path_points else 0.0)
        self._y = well_y + (path_points[0][1] if path_points else 0.0)
        self._z = settings.print_z_height  # already at print height
        self._issues = []
        self._settings = settings

        flow = getattr(settings, 'flow_rate', 0.01) or 0.01

        # Starting waypoint
        self._wp(segment="start", well=well_name)

        # Prime
        prime_mm = (settings.get_retract_amount(pump_id)
                    if hasattr(settings, 'get_retract_amount') else 0)
        if prime_mm > 0:
            self._move_pump(pump_id, prime_mm, settings.pump_feedrate,
                            segment="prime", well=well_name)

        # Print layers
        for layer in range(settings.num_layers):
            if layer > 0:
                self._move_z(self._z + settings.layer_height,
                             settings.z_feedrate, segment="print",
                             well=well_name)

            self._print_path_coordinated(
                path_points, well_x, well_y, pump_id, flow,
                settings.print_feedrate, settings, well_name)

        # Retract
        retract_mm = (settings.get_retract_amount(pump_id)
                      if hasattr(settings, 'get_retract_amount') else 0)
        if retract_mm > 0:
            self._move_pump(pump_id, -retract_mm, settings.pump_feedrate,
                            segment="retract", well=well_name)

        # End waypoint
        self._wp(segment="end", well=well_name)

        return TrajectoryResult(
            waypoints=self._waypoints,
            valid=len(self._waypoints) > 1 and not self._issues,
            issues=self._issues,
            total_duration_s=self._t,
            well_count=1,
        )

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
        # v7.5.x SAFETY: this planner's Z geometry assumes larger Z = higher.
        # On a ZDIR=-1 machine (needle descends as raw Z increases) the
        # raise-before-travel gates invert and "above the well" lands below it,
        # so the retract-before-cross-position-XY guarantee does NOT hold here.
        # Warn loudly; the discrete / hybrid-direct path should be used instead.
        try:
            from SupportClasses.StageController import ZDIR as _ZDIR
            if _ZDIR != 1:
                logger.warning(
                    "PrintTrajectoryPlanner: ZDIR=%s (needle descends as raw Z "
                    "increases). This planner's Z model assumes ZDIR=+1 — the "
                    "trajectory Z-raise-before-travel may be UNSAFE on this "
                    "machine. Use the discrete/hybrid-direct print path.", _ZDIR)
        except Exception:
            pass

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
        # v7.5.x: capture settings so _well_xy can prefer the CALIBRATED
        # well-position map over the geometric offset.
        self._settings = settings
        try:
            _s = getattr(settings, "plate_axis_sign", (1.0, 1.0))
            self._plate_axis_sign = (float(_s[0]), float(_s[1]))
        except Exception:
            self._plate_axis_sign = (1.0, 1.0)

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
                # PRINT calls _do_print_wells directly.
                # Service steps were already executed as their own entries.
                target_wells = getattr(step, 'target_wells', [])
                flow = getattr(settings, 'flow_rate', 0.01) or 0.01
                if target_wells:
                    self._do_print_wells(
                        plate, target_wells, pump, path_points,
                        flow, settings)
                    well_count += len(target_wells)

            elif stype == PlanStepType.RETURN_HOME:
                self._next_segment()
                self._move_z(settings.travel_z_height, settings.z_feedrate,
                             segment="travel")
                _rh_travel_fr = getattr(settings, 'travel_speed_mm_s', 10.0) * 60.0
                self._move_xy(0, 0, _rh_travel_fr, segment='travel')
                self._wp(segment="end")

            elif stype == PlanStepType.WASTE:
                self._do_waste(plate, well_model, pump, settings)
            elif stype == PlanStepType.WASH:
                self._do_wash(plate, well_model, settings)
            elif stype == PlanStepType.REFILL_BUFFER:
                self._do_buffer(plate, well_model, pump, settings)
            elif stype == PlanStepType.LOAD_INK:
                vol = step.volume_uL if step.volume_uL > 0 else 50.0
                self._do_load_ink(plate, well_model, pump, vol, settings)

            # ── v7.2.9: New step types ────────────────────────────
            elif stype == PlanStepType.GATHER_INK:
                # Same as LOAD_INK but volume already includes extra %
                vol = step.volume_uL if step.volume_uL > 0 else 50.0
                self._do_load_ink(plate, well_model, pump, vol, settings)

            elif stype == PlanStepType.MOVE_SAFE_Z:
                # Raise to safe Z for fast travel
                self._next_segment()
                safe_z = settings.travel_z_height
                if self._z < safe_z - 0.01:
                    fast_z = getattr(settings, 'fast_z_feedrate_mm_min',
                                     settings.z_feedrate)
                    self._move_z(safe_z, fast_z, segment="travel")

            elif stype == PlanStepType.TRAVEL_XY:
                # Fast XY travel to target well
                self._next_segment()
                target_wells = getattr(step, 'target_wells', [])
                if target_wells:
                    try:
                        wx, wy = self._well_xy(plate, target_wells[0])
                        speed = getattr(step, 'travel_speed_mm_s', 10.0)
                        self._move_xy(wx, wy, speed * 60.0,
                                      segment="travel",
                                      well=target_wells[0])
                    except Exception:
                        self._issues.append(
                            f"TRAVEL_XY: can't resolve {target_wells[0]}")

            elif stype == PlanStepType.FINAL_CLEANUP:
                # End-of-print cleanup: waste + wash as configured
                self._next_segment()
                sub_steps = getattr(step, 'sub_steps', [])
                if "waste" in sub_steps:
                    self._do_waste(plate, well_model, pump, settings)
                if "wash" in sub_steps:
                    self._do_wash(plate, well_model, settings)

            elif stype == PlanStepType.INK_SWAP:
                # Full ink swap sequence — execute sub_steps in order
                self._next_segment()
                sub_steps = getattr(step, 'sub_steps', [])
                for sub in sub_steps:
                    if sub == "waste":
                        self._do_waste(plate, well_model, pump, settings)
                    elif sub == "wash":
                        self._do_wash(plate, well_model, settings)
                    elif sub == "buffer":
                        self._do_buffer(plate, well_model, pump, settings)
                    elif sub == "ink_load":
                        vol = step.volume_uL if step.volume_uL > 0 else 50.0
                        self._do_load_ink(
                            plate, well_model, pump, vol, settings)

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

def resolve_well_xy_mm(well_name, plate, settings):
    """Well centre in ZERO-REF mm — the single well→stage resolver for the
    print path (the frame ``move_xy_absolute(from_zero_ref=True)`` and the
    trajectory waypoints expect).

    Prefers the per-job CALIBRATED map stamped on ``settings.well_positions_mm``
    (taught/warped absolute stage positions already converted to zero-ref mm at
    job-build time). Falls back to the GEOMETRIC plate-local offset
    (``plate.get_well_position`` — A1 at origin) mapped onto the stage axes by
    ``settings.plate_axis_sign`` when the well has no calibration. The geometric
    branch is byte-identical to the legacy behaviour, so uncalibrated jobs and
    tests are unaffected. A malformed sign degrades to aligned ``(1, 1)``."""
    cal = getattr(settings, "well_positions_mm", None)
    if cal:
        p = cal.get(well_name)
        if p is None and isinstance(well_name, str):
            p = cal.get(well_name.upper())
        if p is not None:
            try:
                return (float(p[0]), float(p[1]))
            except (TypeError, ValueError, IndexError):
                pass
    wx, wy = plate.get_well_position(well_name)
    try:
        sgn = getattr(settings, "plate_axis_sign", (1.0, 1.0))
        return (float(sgn[0]) * wx, float(sgn[1]) * wy)
    except (TypeError, ValueError, IndexError):
        return (wx, wy)


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
