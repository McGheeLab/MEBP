"""
Print Manager - Print file loading, path execution, and job control.

Handles:
- Loading print files (custom JSON format or simplified G-code)
- Generating print paths from well plate + pattern combinations
- Sequential execution with pause / resume / abort
- Progress reporting via callback

Session 4 additions:
- SWITCH_PUMP command type for multi-material printing (Task 1)
- Per-pump retract/prime amounts (Task 1)
- Position logging during print execution (Task 2)
- PrintQueue for sequential multi-job execution (Task 6)

Enhancement additions:
- G-code export: export_gcode() for compatibility with external slicers
- Print resume: save/load print progress for crash recovery
- Print history: integration with PrintHistory for persistent logging

v7.1 Session I additions:
- TRAJECTORY command + TrajectoryExecutor for smooth trajectory tracking (P8.1–P8.3)
- PrintRecorder auto-start/stop for execution recording (P8.4)
- WorkspaceConfig reference on PrintJob (P8.5)
- SERVICE_SEQUENCE command + ServiceSequenceExecutor (P8.6)
- FluidColumnTracker for ink state management (P8.7)
- Ink change detection in SWITCH_PUMP (P8.8)
- Incremental vs continuous mode tracking (P8.9)

Print File Format (JSON):
{
    "name": "My Print Job",
    "description": "...",
    "settings": {
        "xy_feedrate": 1000,
        "z_feedrate": 60,
        "print_feedrate": 200,
        "travel_z_height": 5.0,
        "print_z_height": 0.1,
        "layer_height": 0.1,
        "num_layers": 1,
        "retract_amounts": {"P1": 0.1, "P2": 0.1, "P3": 0.0},
        "prime_amounts": {"P1": 0.05, "P2": 0.05, "P3": 0.0}
    },
    "commands": [
        {"type": "move_xy", "x": 10.0, "y": 20.0},
        {"type": "move_z", "z": 0.1},
        {"type": "extrude", "pump": "P1", "amount": 0.5, "feedrate": 30},
        {"type": "print_path", "points": [[0,0],[10,0],[10,10]], "pump": "P1", "flow_rate": 0.01},
        {"type": "switch_pump", "pump": "P2"},
        {"type": "dwell", "seconds": 1.0},
        {"type": "travel_z_up"},
        {"type": "travel_z_down"},
        ...
    ]
}

Usage:
    manager = PrintManager(stage_controller)
    manager.load_file("job.json")
    manager.on_progress = my_callback  # (current_step, total_steps, message)
    manager.start()
    manager.pause()
    manager.resume()
    manager.abort()
"""

from __future__ import annotations

import json
import math
import time
import threading
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Callable, Optional

from SupportClasses.ZPStage import AXIS_MAP
from SupportClasses.PrintExecutionLogger import PrintExecutionLogger

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Data Structures
# ═══════════════════════════════════════════════════════════════════

class PrintState(Enum):
    """Current state of the print job."""
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    ABORTED = auto()
    ERROR = auto()


class CommandType(Enum):
    """Types of print commands."""
    MOVE_XY = "move_xy"             # Move XY to absolute position (relative to zero ref)
    MOVE_Z = "move_z"               # Move Z to absolute height
    MOVE_Z_REL = "move_z_rel"       # Move Z by relative amount
    # v7.5.x: DISPENSE = push fluid OUT (a relative pump move; + volume =
    # dispense, − = aspirate — see StageController.move_pump_uL). Renamed from
    # EXTRUDE; the value stays "extrude" so persisted exec-log manifests stay
    # comparable.
    DISPENSE = "extrude"            # Pump move (+ dispense / − aspirate), relative
    PRINT_PATH = "print_path"       # Coordinated XY move + dispense
    DWELL = "dwell"                 # Wait for specified time
    TRAVEL_UP = "travel_up"         # Raise Z to travel height
    TRAVEL_DOWN = "travel_down"     # Lower Z to print height
    SET_PUMP_RATE = "set_pump_rate" # Set pump flow rate for upcoming moves
    HOME_XY = "home_xy"             # Move XY to zero reference
    COMMENT = "comment"             # No-op, just a label/comment
    SWITCH_PUMP = "switch_pump"     # Session 4: Switch active pump (multi-material)
    # v7.1 additions
    TRAJECTORY = "trajectory"       # P8.1: Execute full trajectory via MotionController
    SERVICE_SEQUENCE = "service_seq"  # P8.6: Run service sequence (waste→wash→buffer→ink)


@dataclass
class PrintCommand:
    """A single command in a print job."""
    type: CommandType
    params: dict = field(default_factory=dict)
    label: str = ""  # Human-readable description

    def __repr__(self):
        return f"PrintCommand({self.type.value}, {self.params}, '{self.label}')"


# ── Arc-length path math for velocity-following printing (pure, testable) ──
# The velocity follower (PrintManager._execute_print_path_velocity) parametrizes
# the toolpath by ARC LENGTH, not wall-clock time: the "carrot" the stage chases
# is placed a fixed lookahead distance ahead of the stage's ACTUAL measured
# progress along the path. So a stage that runs slower than commanded simply
# advances the carrot slower — it can NEVER accumulate lag/run ahead (the failure
# mode of the open-loop time-paced path), and the pump deposits volume in
# proportion to real distance travelled, so the bead stays correct at any speed.

# v7.5.x: the arc-length helpers now live in SupportClasses/VelocityControl.py
# (their canonical home, shared with the XY-Challenge bench so the tuned control
# law is identical). Re-exported here so existing
# ``from SupportClasses.PrintManager import polyline_arclength, …`` imports keep
# working unchanged.
from SupportClasses.VelocityControl import (   # noqa: E402
    polyline_arclength, point_at_arclength, tangent_at_arclength,
    project_on_polyline,
)
from SupportClasses import VelocityControl as _velctl   # noqa: E402


@dataclass
class PrintSettings:
    """Print job settings."""
    xy_feedrate: float = 1000.0      # XY travel speed (stage units/sec)
    z_feedrate: float = 60.0         # Z feedrate (mm/min)
    print_feedrate: float = 200.0    # XY speed during printing
    pump_feedrate: float = 30.0      # Pump feedrate (mm/min)
    travel_z_height: float = 5.0     # Z height for travel moves (mm above zero)
    print_z_height: float = 0.1      # Z height for printing (mm above zero)
    layer_height: float = 0.1        # Height increment per layer
    num_layers: int = 1              # Number of layers
    # v7.5.x: print-Z up-direction (zero-ref frame), stamped from
    # StageController.print_z_dir() at job-build time. +1 = "up is larger Z"
    # (conventional); -1 = "up is smaller Z" (ME3B V1, needle descends as raw Z
    # increases). Layer build-up steps by `z_up_sign * layer_height` so layers
    # grow UP on either polarity. Default +1 preserves legacy additive behaviour.
    z_up_sign: float = 1.0
    # v7.5.x: per-machine plate-local→stage axis sign, stamped from
    # StageController.plate_axis_sign() at job-build time. (-1, -1) when the
    # plate is mounted 180° to the stage (ME3B V1) so a GEOMETRIC well centre
    # (plate.get_well_position, A1-relative mm) maps to the physically-correct
    # well. (1, 1) = aligned (legacy). Applied only to geometric well centres,
    # never to already-taught/calibrated positions.
    plate_axis_sign: tuple = (1.0, 1.0)
    # v7.5.x: CALIBRATED well centres in ZERO-REF mm, keyed by well name,
    # stamped at job-build time from the plate calibration (taught/warped
    # absolute stage positions converted to zero-ref mm). The execution path
    # (discrete job + HybridPlanExecutor + PrintTrajectoryPlanner) PREFERS these
    # over the GEOMETRIC plate-local offset so the needle reaches the
    # physically-taught well — not a grid anchored at the stage origin (which,
    # under plate_axis_sign=(-1,-1), lands on negative coords that clamp to 0,0).
    # None = no calibration → geometric × plate_axis_sign fallback (legacy
    # behaviour; uncalibrated jobs / tests unaffected). See
    # PrintTrajectoryPlanner.resolve_well_xy_mm.
    well_positions_mm: dict | None = None
    # v7.5.x: the plate bottom is flat but TILTED. `print_z_height` above is the
    # print Z resolved at ONE point (the taught plate-bottom anchor); this
    # optional plane adds the measured gradient so each well prints at its OWN
    # local plate bottom. Stamped at build time from
    # StageController.plate_z_plane_for_job() — a job carries per-machine facts
    # (see `z_up_sign` / `plate_axis_sign` / `well_positions_mm` above) so a run
    # is reproducible from the job and a re-teach between build and run cannot
    # silently change a RESUMED print's Z.
    #
    # Frame: XY in ZERO-REF mm (the frame the print path works in), Z in zero-ref
    # mm, slopes mm/mm. Keys are `x0_mm`/`y0_mm` — deliberately NOT the live
    # plane's `x0_um`/`y0_um`, so mixing the two forms raises instead of silently
    # computing a 1000×-wrong offset. Evaluate ONLY via
    # SupportClasses.PlateZPlane.job_plane_z_zref_mm.
    #
    # None (the default) = no tilt correction ⇒ every well uses `print_z_height`,
    # byte-identical to the legacy plan.
    plate_z_plane_zref_mm: dict | None = None
    # v7.5.x: the operator's INTENT — desired height above the plate bottom (mm).
    # Required alongside the plane: without it a per-well Z could only *shift*
    # `print_z_height`, whereas with it each well's Z is computed exactly the way
    # the anchor value was (local plate bottom + this height). None = unknown ⇒
    # the plane is ignored and `print_z_height` is used as-is.
    print_height_above_bottom_mm: float | None = None
    retract_amount: float = 0.0      # Pump retraction after path segment (legacy single-pump)
    prime_amount: float = 0.0        # Pump prime before path segment (legacy single-pump)
    dwell_after_move: float = 0.0    # Seconds to wait after travel moves

    # Session 4: Per-pump retract/prime amounts
    retract_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})

    # v7.5.x: small lift between objects printed in the SAME well (mm above
    # print Z). Just enough to clear thin printed material on an inter-object
    # hop — NOT the full travel retract. Applied in the polarity-safe height
    # frame; floored by the plate-insert clearance.
    #
    # This is ALSO the operator-facing "retract height after each print line" in
    # Quick Print: each sub-path/stroke is its own PRINT_PATH (see
    # ``build_well_plate_job(path_segments=...)``), and between strokes the
    # needle lifts to ``print Z + intra_well_hop_z_mm`` before the XY move.
    intra_well_hop_z_mm: float = 1.0
    # v7.5.x: optional FAST move speeds for the inter-line/inter-object hop only
    # (the lift up, the XY travel to the next stroke, and the lower back down).
    # 0 = use the defaults (Z: the controller retract/insert feedrate; XY:
    # ``travel_speed_mm_s``). Set by Quick Print's "Line-move Z/XY speed" knobs.
    # The first object's full approach (TRAVEL_UP/MOVE_XY/MOVE_Z) is NOT affected
    # — only the per-segment hop after a printed line.
    line_move_z_speed_mm_s: float = 0.0   # mm/s; 0 = default
    line_move_xy_speed_mm_s: float = 0.0  # mm/s; 0 = default

    # v7.2: µL-based pump settings
    pump_rate_uL_s: float = 0.25         # Default pump flow rate (µL/s)
    # v7.2.7: print_speed_mm_s — explicit mm/s speed from GUI
    print_speed_mm_s: float = 5.0        # XY speed during printing (mm/s)
    travel_speed_mm_s: float = 10.0       # XY speed during travel (mm/s)

    retract_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts_uL: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    pump_rates_uL_s: dict = field(default_factory=lambda: {"P1": 0.25, "P2": 0.25, "P3": 0.25})

    # v7.2.6-auto: top_z_height — calibration-derived Z heights and motion tiers
    top_z_height: float = 0.0           # plate top surface (from calibration top_z)
    fast_z_feedrate_mm_min: float = 120.0   # Z speed above well top (service/travel)
    entry_z_feedrate_mm_min: float = 12.0   # Z speed entering well (slow, safe)
    service_xy_speed_mm_s: float = 50.0     # XY speed for service moves (waste/wash/ink)
    auto_pump_rate_uL_s: float = 0.0        # print pump rate from extrusion physics
    service_pump_rate_uL_s: float = 5.0     # service pump rate (max for gauge)

    # v7.5.x: CONFIRMED per-segment printing. The default PRINT_PATH is streamed
    # OPEN-LOOP — each segment's XY move is fire-and-forget, paced only by a
    # time.sleep(seg_len/print_speed). On a stage whose real throughput is slower
    # than the commanded speed (e.g. Prior SMS% mis-calibrated, or short accel-
    # dominated segments), the physical stage falls PROGRESSIVELY behind the
    # commanded stream (observed lag grew to ~6-10 mm across a ~10 mm print) while
    # the pump keeps extruding on the fast software schedule → the deposited
    # pattern smears/distorts. When True, _execute_print_path instead WAITS for
    # the stage to physically arrive (± _SEGMENT_SETTLE_TOL_UM) and drains the
    # pump board (M400) after EACH segment, so the stream can never outrun the
    # stage and geometry stays correct at any speed (trade-off: the print runs at
    # the stage's true throughput, which may be slower). Set by Quick Print.
    confirm_each_segment: bool = False

    # v7.5.x: CLOSED-LOOP velocity-following printing (preferred over the
    # stop-and-go confirm_each_segment). Instead of streaming discrete moves, a
    # real-time control loop continuously polls the stage's ACTUAL position and
    # re-commands a velocity vector (Prior ``VS``) toward a carrot placed a fixed
    # lookahead ahead of the stage's real progress ALONG the path (pure pursuit,
    # arc-length parametrized). The stage therefore moves smoothly/continuously,
    # position feedback corrects any drift, the carrot can never run ahead of a
    # slow stage (no accumulating lag), and the pump deposits volume in
    # proportion to real distance travelled (bead correct at any speed). Requires
    # a stage with a continuous-velocity command (Prior ``VS`` /
    # ``send_velocity_xy``); when unavailable it falls back to the discrete path.
    # See PrintManager._execute_print_path_velocity. Set by Quick Print.
    velocity_follow: bool = False

    # v7.5.x: OPEN-LOOP velocity streaming ("open_loop" motion mode). The
    # controller is fed a continuous velocity vector along the path TANGENT,
    # updated as the path direction changes, with the target advancing by
    # wall-clock time at the commanded speed — i.e. it "just updates the
    # velocity vector needed" rather than moving to prescribed points (that is
    # the confirm_each_segment mode). No position feedback (feed-forward):
    # smooth continuous velocity for even ink laydown, but no drift correction
    # and no runaway guard (the closed-loop velocity_follow adds those). The
    # pump deposits ∝ the time-based target advance. Requires a continuous-
    # velocity command (Prior ``VS`` / ``send_velocity_xy``); when unavailable
    # it falls back to the discrete point-stream path. See
    # PrintManager._execute_print_path_open_velocity. Set by Quick Print.
    velocity_open_loop: bool = False

    # v7.5.x: per-mode path-following parameters, tuned by the XY Printing
    # Challenge and stamped from PrintTimingCalibrationStore by Quick Print.
    # Defaults preserve legacy behaviour (open-loop: no pace change; confirm +
    # velocity: 0 → the executor's class-constant tuning), so a job that doesn't
    # set them behaves exactly as before.
    #  • open-loop: multiply each segment's XY pacing sleep by this so the loop
    #    paces to the stage's MEASURED effective speed (>1 for a slow stage).
    pace_correction: float = 1.0
    #  • confirmed per-segment: arrival tolerance (µm). 0 = class default.
    segment_settle_tol_um: float = 0.0
    #  • velocity follower pure-pursuit tuning. 0 = class defaults.
    vel_lookahead_mm: float = 0.0
    vel_control_hz: float = 0.0
    vel_decel_mm: float = 0.0

    # v7.5.x: MACHINE-MEASURED calibration consumed by the closed-loop velocity
    # follower (stamped from PrintTimingCalibrationStore by Quick Print; 0 =
    # unmeasured → the follower falls back to safety_limits / class constants).
    #  • xy_max_speed_um_s: true top speed at 100% SMS — sets SMS + the VS clamp
    #    so commanded velocity is achievable AND not clamped below it.
    #  • control_loop_ms: measured closed-loop period → control rate + the dead-
    #    time speed cap (v ≤ lookahead/(loop·safety)) that kills the overshoot
    #    limit-cycle ("back-and-forth").
    #  • phase_lag_s: measured stage lag behind commands → widens the dead-time
    #    used for the speed cap / minimum lookahead.
    xy_max_speed_um_s: float = 0.0
    control_loop_ms: float = 0.0
    phase_lag_s: float = 0.0

    # v7.5.x: XY controller acceleration % applied before a velocity-mode path
    # (was read via getattr with an 80 fallback; now a declared field).
    xy_accel_pct: float = 80.0

    # v7.5.x: velocity-follower corner-aware speed scheduling + cross-track PID.
    #  • vel_corner_angle_deg: turn angle (deg) that counts as a corner to slow
    #    into. 0 = class default.
    #  • vel_corner_speed_factor: fraction of print speed allowed AT a sharp
    #    corner (0..1). 0 = class default (no extra corner slowdown beyond the
    #    factor default).
    #  • vel_pid_kp / vel_pid_kd: cross-track PID gains (perpendicular pull back
    #    onto the path). 0 = pure pursuit (legacy).
    vel_corner_angle_deg: float = 0.0
    vel_corner_speed_factor: float = 0.0
    vel_pid_kp: float = 0.0
    vel_pid_kd: float = 0.0

    # v7.5.x: confirmed-per-segment — turn angle that counts as a corner. The
    # executor waits/drains ONLY at corners (straight edges stream). 0 = class
    # default.
    confirm_corner_angle_deg: float = 0.0

    # v7.5.x (XY-Challenge upgrade): the FULL velocity-follower tuning dict,
    # copied verbatim from PrintTimingCalibrationStore's "velocity" bucket.
    #
    # One dict instead of ~25 more scalar fields: the individual named fields
    # above stay as the fallback (so nothing that reads them breaks), but every
    # NEW tunable arrives here, which means adding one needs no PrintSettings
    # change at all. Safe by construction — PrintSettings already carries five
    # dict fields with default_factory, save_print_progress JSON-dumps every
    # field, and from_dict filters by field name.
    #
    # Empty = use the named fields / class defaults = legacy behaviour.
    vel_tuning: dict = field(default_factory=dict)

    # Prior SCS S-curve jerk limit (1..100), 0 = don't touch. Changing it
    # invalidates the measured control_loop_ms / dead time / top speed, so it is
    # stamped into the print log for attribution.
    xy_jerk_pct: float = 0.0

    # ── v7.6: feature-aware FEED PLAN (SupportClasses/XYFeedPlan.py) ──
    #
    # The velocity follower with ONE fixed tuning provably cannot hold a
    # resolution element through a corner: pure pursuit cuts every corner by
    # ≈0.4·lookahead (~220 µm measured on ME3B V1 at any speed-stable
    # lookahead) and cannot turn a 180° reversal at all. The feed plan instead
    # SPLITS the path at sharp corners/reversals, stops on those vertices, and
    # sizes each section's lookahead/speed from its own curvature — which
    # measured 18/18 geometry-panel shapes ≤ 30 µm on real hardware where the
    # single tuning managed 0/18.
    #
    # All default 0/False = absent = the legacy single-tuning follower, so an
    # unstamped or uncalibrated job is byte-identical.
    feed_plan_enabled: bool = False        # velocity mode only
    feed_plan_element_um: float = 0.0      # resolution element; 0 → 30 µm
    feed_plan_corner_split_deg: float = 0.0  # 0 → XYFeedPlan.CORNER_SPLIT_DEG
    #: v7.7 corner policy — "slow" (default) slows through corners and keeps the
    #: pump advancing for the whole path; "stop" is the v7.6 sectioned plan that
    #: halts on each sharp vertex. Both hold the resolution element in
    #: simulation (within ~1 µm); "stop" is ~1.5–1.9× faster on corner-heavy
    #: geometry, "slow" never interrupts deposition. A near-180° reversal stops
    #: under either policy — a cusp cannot be traversed at any speed.
    feed_plan_corner_policy: str = "slow"

    def velocity_tuning(self) -> dict:
        """The effective velocity-follower tuning: ``vel_tuning`` when stamped,
        else the legacy named fields. One accessor so the print path and the
        bench cannot read it differently."""
        if isinstance(self.vel_tuning, dict) and self.vel_tuning:
            return dict(self.vel_tuning)
        return {
            "lookahead_mm": self.vel_lookahead_mm,
            "control_hz": self.vel_control_hz,
            "decel_mm": self.vel_decel_mm,
            "corner_angle_deg": self.vel_corner_angle_deg,
            "corner_speed_factor": self.vel_corner_speed_factor,
            "pid_kp": self.vel_pid_kp,
            "pid_kd": self.vel_pid_kd,
        }

    def get_retract_uL(self, pump: str) -> float:
        """Get retract amount for a pump in µL (v7.2). Falls back to legacy mm value."""
        uL = self.retract_amounts_uL.get(pump, 0.0)
        if uL > 0:
            return uL
        return 0.0

    def get_prime_uL(self, pump: str) -> float:
        """Get prime amount for a pump in µL (v7.2). Falls back to legacy mm value."""
        uL = self.prime_amounts_uL.get(pump, 0.0)
        if uL > 0:
            return uL
        return 0.0

    def get_pump_rate(self, pump: str) -> float:
        """Get flow rate for a specific pump in µL/s."""
        return self.pump_rates_uL_s.get(pump, self.pump_rate_uL_s)


    @classmethod
    def from_dict(cls, d: dict) -> "PrintSettings":
        """Create from dictionary, ignoring unknown keys."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in d.items() if k in valid_keys}
        instance = cls(**filtered)

        # v7.2.7: infer mm/s from legacy print_feedrate if not set

        if getattr(instance, 'print_speed_mm_s', 0) <= 0 and instance.print_feedrate > 0:

            instance.print_speed_mm_s = instance.print_feedrate / 60.0

        if getattr(instance, 'travel_speed_mm_s', 0) <= 0 and instance.xy_feedrate > 0:

            # xy_feedrate could be mm/s (new GUI) or old stage-units/s

            if instance.xy_feedrate < 100:  # likely mm/s

                instance.travel_speed_mm_s = instance.xy_feedrate

            else:  # likely legacy stage units

                instance.travel_speed_mm_s = instance.xy_feedrate / 1000.0

        return instance

    def get_retract_amount(self, pump: str) -> float:
        """Get retract amount for a specific pump (falls back to global)."""
        per_pump = self.retract_amounts.get(pump, None)
        if per_pump is not None and per_pump > 0:
            return per_pump
        return self.retract_amount

    def get_prime_amount(self, pump: str) -> float:
        """Get prime amount for a specific pump (falls back to global)."""
        per_pump = self.prime_amounts.get(pump, None)
        if per_pump is not None and per_pump > 0:
            return per_pump
        return self.prime_amount


@dataclass
class PrintJob:
    """A complete print job with metadata, settings, and commands."""
    name: str = "Untitled"
    description: str = ""
    settings: PrintSettings = field(default_factory=PrintSettings)
    commands: list[PrintCommand] = field(default_factory=list)
    source_file: str = ""

    # v7.1 P8.5: WorkspaceConfig reference for trajectory-based prints
    # Stores needle/syringe/ink/plate config used to generate this job.
    # Set to None for legacy discrete-command jobs.
    workspace: object = None          # WorkspaceConfig (typed loosely to avoid circular import)
    well_setup: object = None         # WellSetupModel reference (same reason)
    trajectory_waypoints: list = field(default_factory=list)  # Waypoint list for trajectory mode

    # v7.2.9: Hybrid execution — plan steps drive execution, not waypoints
    plan_of_action: object = None     # PrintPlanOfAction
    plate: object = None              # WellPlate for well position lookups
    path_points: list = field(default_factory=list)  # Print geometry [(x,y), ...]
    hw_config: object = None          # HardwareConfig
    estimated_duration_s: float = 0.0  # Hybrid executor time estimate

    @property
    def total_steps(self) -> int:
        return len(self.commands)

    def get_xy_path(self) -> list[tuple[float, float]]:
        """
        Extract all XY coordinates from the job for preview rendering.
        Returns list of (x, y) points in order.
        """
        points = []
        for cmd in self.commands:
            if cmd.type == CommandType.MOVE_XY:
                points.append((cmd.params.get("x", 0), cmd.params.get("y", 0)))
            elif cmd.type == CommandType.PRINT_PATH:
                path = cmd.params.get("points", [])
                points.extend((p[0], p[1]) for p in path)
            elif cmd.type == CommandType.HOME_XY:
                points.append((0, 0))
        return points

    def get_path_segments(self) -> list[dict]:
        """
        Extract path segments with type info for colored preview rendering.
        
        Returns list of dicts:
            {"type": "travel"|"print", "points": [(x,y), ...]}
        """
        segments = []
        current_pos = (0, 0)

        for cmd in self.commands:
            if cmd.type == CommandType.MOVE_XY:
                x = cmd.params.get("x", current_pos[0])
                y = cmd.params.get("y", current_pos[1])
                target = (x, y)
                if target != current_pos:
                    segments.append({
                        "type": "travel",
                        "points": [current_pos, target],
                    })
                current_pos = target

            elif cmd.type == CommandType.PRINT_PATH:
                path = cmd.params.get("points", [])
                if path:
                    # Travel to first point of path if needed
                    first = (path[0][0], path[0][1])
                    if first != current_pos:
                        segments.append({
                            "type": "travel",
                            "points": [current_pos, first],
                        })
                    # Print path
                    segments.append({
                        "type": "print",
                        "points": [(p[0], p[1]) for p in path],
                        "pump": cmd.params.get("pump", "P1"),
                    })
                    current_pos = (path[-1][0], path[-1][1])

            elif cmd.type == CommandType.HOME_XY:
                if current_pos != (0, 0):
                    segments.append({
                        "type": "travel",
                        "points": [current_pos, (0, 0)],
                    })
                current_pos = (0, 0)

        return segments

    def get_layer_boundaries(self) -> list[tuple[int, str]]:
        """
        Parse layer boundaries from COMMENT commands.
        
        Returns list of (command_index, layer_label) for each layer start.
        """
        boundaries = []
        for i, cmd in enumerate(self.commands):
            if cmd.type == CommandType.COMMENT and "Layer" in cmd.label:
                boundaries.append((i, cmd.label))
        return boundaries


# ═══════════════════════════════════════════════════════════════════
# File Loading
# ═══════════════════════════════════════════════════════════════════

def load_print_file(filepath: str) -> PrintJob:
    """
    Load a print job from a file.
    
    Supports:
        .json  - Custom JSON print format
        .gcode - Simplified G-code (XY/Z moves only, no temp commands)
    """
    path = Path(filepath)

    if path.suffix.lower() == ".json":
        return _load_json_file(path)
    elif path.suffix.lower() in (".gcode", ".gco", ".nc"):
        return _load_gcode_file(path)
    else:
        raise ValueError(f"Unsupported file format: {path.suffix}")


def _load_json_file(path: Path) -> PrintJob:
    """Load from custom JSON format. Auto-detects v7.1 vs v7.2 format."""
    with open(path, "r") as f:
        data = json.load(f)

    # v7.2: Detect file version and migrate if needed
    file_version = data.get("version", "7.1")
    if file_version < "7.2":
        logger.info(f"Loading v{file_version} print file — commands may use legacy mm format")

    settings = PrintSettings.from_dict(data.get("settings", {}))
    commands = []

    for cmd_data in data.get("commands", []):
        cmd_type_str = cmd_data.get("type", "")
        try:
            cmd_type = CommandType(cmd_type_str)
        except ValueError:
            logger.warning(f"Unknown command type: {cmd_type_str}, skipping")
            continue

        params = {k: v for k, v in cmd_data.items() if k != "type"}
        label = params.pop("label", "")
        commands.append(PrintCommand(type=cmd_type, params=params, label=label))

    return PrintJob(
        name=data.get("name", path.stem),
        description=data.get("description", ""),
        settings=settings,
        commands=commands,
        source_file=str(path),
    )


def _load_gcode_file(path: Path) -> PrintJob:
    """
    Load from G-code file. Converts to PrintCommands.
    Only processes G0/G1 (moves), G4 (dwell), G28 (home).
    Ignores temperature, fan, and other printer-specific commands.
    """
    commands = []
    current_x, current_y, current_z = 0.0, 0.0, 0.0
    current_e = 0.0

    with open(path, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith(";"):
                if line.startswith(";"):
                    commands.append(PrintCommand(
                        type=CommandType.COMMENT,
                        label=line[1:].strip(),
                    ))
                continue

            if ";" in line:
                line = line[:line.index(";")].strip()

            parts = line.split()
            if not parts:
                continue

            gcode = parts[0].upper()

            if gcode in ("G0", "G1"):
                params = _parse_gcode_params(parts[1:])
                has_xy = "X" in params or "Y" in params
                has_z = "Z" in params
                has_e = "E" in params

                new_x = params.get("X", current_x)
                new_y = params.get("Y", current_y)
                new_z = params.get("Z", current_z)
                new_e = params.get("E", current_e)

                if has_z and not has_xy:
                    commands.append(PrintCommand(
                        type=CommandType.MOVE_Z,
                        params={"z": new_z},
                        label=f"G-code Z move (line {line_num})",
                    ))

                if has_xy:
                    if has_e and new_e != current_e:
                        path_points = [(current_x, current_y), (new_x, new_y)]
                        commands.append(PrintCommand(
                            type=CommandType.PRINT_PATH,
                            params={
                                "points": path_points,
                                "pump": "P1",
                                "flow_rate": abs(new_e - current_e) /
                                    max(math.sqrt((new_x - current_x)**2 + (new_y - current_y)**2), 0.001),
                            },
                            label=f"G-code print move (line {line_num})",
                        ))
                    else:
                        commands.append(PrintCommand(
                            type=CommandType.MOVE_XY,
                            params={"x": new_x, "y": new_y},
                            label=f"G-code travel (line {line_num})",
                        ))

                current_x, current_y, current_z, current_e = new_x, new_y, new_z, new_e

            elif gcode == "G4":
                params = _parse_gcode_params(parts[1:])
                seconds = params.get("S", params.get("P", 0) / 1000)
                commands.append(PrintCommand(
                    type=CommandType.DWELL,
                    params={"seconds": seconds},
                    label=f"G-code dwell (line {line_num})",
                ))

            elif gcode == "G28":
                commands.append(PrintCommand(
                    type=CommandType.HOME_XY,
                    label=f"G-code home (line {line_num})",
                ))
                current_x, current_y = 0, 0

    return PrintJob(
        name=path.stem,
        description=f"Imported from G-code: {path.name}",
        settings=PrintSettings(),
        commands=commands,
        source_file=str(path),
    )


def well_print_z_zref_mm(settings, well_x_mm: float, well_y_mm: float,
                         layer: int = 0) -> float:
    """Print Z (zero-ref mm) for one well on one layer.

    The plate bottom is flat but tilted, so the print Z that gives the intended
    standoff differs from well to well. When the job carries a tilt plane AND the
    operator's intended height above the plate bottom, each well's Z is derived
    from that well's LOCAL plate bottom — computed exactly the way the anchor
    value was, not as a fudge applied to it.

    Without either (the default, and every legacy job) this returns
    ``settings.print_z_height + z_up * layer * layer_height``, i.e. the existing
    plan byte-for-byte.

    ``well_x_mm`` / ``well_y_mm`` are the well centre in ZERO-REF mm — the frame
    ``well_positions_mm`` and ``MOVE_XY`` already use.
    """
    z_up = getattr(settings, "z_up_sign", 1.0)
    base = settings.print_z_height
    plane = getattr(settings, "plate_z_plane_zref_mm", None)
    height = getattr(settings, "print_height_above_bottom_mm", None)
    if plane and height is not None:
        try:
            from SupportClasses.PlateZPlane import job_plane_z_zref_mm
            from SupportClasses.StageController import plate_relative_to_zref
            local_bottom = job_plane_z_zref_mm(plane, well_x_mm, well_y_mm)
            base = plate_relative_to_zref(local_bottom, float(height),
                                          zdir=z_up)
        except Exception as e:
            # A malformed stamped plane must never break a print — fall back to
            # the plate-wide value the job already carries.
            logger.warning("well_print_z_zref_mm: ignoring stamped plate plane "
                           "(%s) — using print_z_height", e)
            base = settings.print_z_height
    return base + z_up * layer * settings.layer_height


def _parse_gcode_params(parts: list[str]) -> dict:
    """Parse G-code parameters like 'X10.5 Y20 Z0.1' into a dict."""
    params = {}
    for part in parts:
        if len(part) >= 2 and part[0].isalpha():
            try:
                params[part[0].upper()] = float(part[1:])
            except ValueError:
                pass
    return params


# ═══════════════════════════════════════════════════════════════════
# Job Builder - Generate print jobs from well plate + patterns
# ═══════════════════════════════════════════════════════════════════

def build_well_plate_job(
    well_positions: list[tuple[str, float, float]],
    path_points: list[tuple[float, float]],
    settings: PrintSettings,
    pump: str = "P1",
    flow_rate: float = 0.01,
    job_name: str = "Well Plate Print",
    pump_sequence: list[str] | None = None,
    pump_per_layer: dict[str, str] | None = None,
    path_segments: list[list[tuple[float, float]]] | None = None,
    return_home: bool = True,
) -> PrintJob:
    """
    Build a print job that prints a pattern in each well of a well plate.

    Session 4: Now supports multi-material via pump_sequence or pump_per_layer.

    Args:
        well_positions: List of (well_name, x, y) for each well to print
        path_points: Pattern points relative to well center (0,0)
        settings: Print settings
        pump: Default pump to use (P1, P2, P3) — used if no multi-material
        flow_rate: Extrusion rate per mm of path travel
        job_name: Name for the job
        pump_sequence: List of pumps to cycle through wells ["P1", "P2"]
        pump_per_layer: Dict mapping layer number (str) to pump {"1": "P1", "2": "P2"}
        path_segments: v7.5.x — OPTIONAL list of independent sub-paths (each a
            list of (x, y) relative to well center). When given, each sub-path
            becomes its own PRINT_PATH with a full lift→travel→lower prologue
            between them, so the needle never drags through already-printed
            material across an inter-object seam (e.g. a saved print made of
            several spirals at different offsets). When None, the single
            ``path_points`` list is used — identical to the prior behavior.
        return_home: v7.5.x — when True (default, legacy behavior) the job ends
            with a final ``TRAVEL_UP`` then ``HOME_XY`` (return to the zero
            reference, i.e. XY 0,0). When False the job still ends with the
            final ``TRAVEL_UP`` (needle retracted out of the well to the travel
            Z) but the ``HOME_XY`` is omitted, so the stage is left where it
            finished printing instead of driving back to 0,0. Quick Print uses
            False (do not slam back to origin).

    Returns:
        PrintJob ready for execution
    """
    commands = []
    active_pump = pump

    for layer in range(settings.num_layers):
        # v7.5.x: step layers along the reference-vector "up" direction so
        # build-up grows away from the plate floor on either Z polarity
        # (legacy additive `+layer*layer_height` drove deeper on ME3B V1).
        z_up = getattr(settings, "z_up_sign", 1.0)
        z_height = settings.print_z_height + z_up * layer * settings.layer_height

        # Determine pump for this layer
        layer_pump = active_pump
        if pump_per_layer:
            layer_key = str(layer + 1)
            if layer_key in pump_per_layer:
                layer_pump = pump_per_layer[layer_key]

        # Insert SWITCH_PUMP if layer pump differs from active pump
        if layer_pump != active_pump:
            commands.append(PrintCommand(
                type=CommandType.SWITCH_PUMP,
                params={"pump": layer_pump, "old_pump": active_pump},
                label=f"Switch from {active_pump} to {layer_pump}",
            ))
            active_pump = layer_pump

        commands.append(PrintCommand(
            type=CommandType.COMMENT,
            label=f"=== Layer {layer + 1}/{settings.num_layers} (z={z_height:.3f}) ===",
        ))

        for well_idx, (well_name, well_x, well_y) in enumerate(well_positions):
            # v7.5.x: resolve THIS well's print Z. With a stamped tilt plane the
            # value tracks the well's local plate bottom; without one it is
            # exactly `z_height` (the plate-wide layer value computed above), so
            # the emitted plan is byte-identical to the legacy one.
            well_z_height = well_print_z_zref_mm(settings, well_x, well_y, layer)

            # Determine pump for this well (pump_sequence overrides)
            well_pump = active_pump
            if pump_sequence and len(pump_sequence) > 0:
                well_pump = pump_sequence[well_idx % len(pump_sequence)]
                if well_pump != active_pump:
                    commands.append(PrintCommand(
                        type=CommandType.SWITCH_PUMP,
                        params={"pump": well_pump, "old_pump": active_pump},
                        label=f"Switch from {active_pump} to {well_pump}",
                    ))
                    active_pump = well_pump

            # v7.5.x: a well may contain MULTIPLE objects (e.g. Quick Print of
            # a saved multi-spiral file). Each object is its own PRINT_PATH with
            # a full lift→travel→lower prologue between them, so the needle never
            # drags through already-printed material across the inter-object
            # seam. Single-object / legacy callers pass path_segments=None and
            # get exactly one segment == path_points (byte-identical plan).
            segments = [s for s in (path_segments or [path_points]) if s]

            # v7.5.x: small intra-well hop height between objects (a few mm
            # apart) — just enough to clear the thin printed material, not a
            # full travel retract. Computed in the polarity-safe HEIGHT frame:
            # `z_up * hop_mm` above the print Z (z_up = -1 on ME3B V1, so this
            # is genuinely "up"/away from the plate on either polarity).
            hop_mm = abs(getattr(settings, "intra_well_hop_z_mm", 1.0))
            hop_z = well_z_height + z_up * hop_mm

            # v7.5.x: optional FAST hop speeds (Quick Print "Line-move Z/XY
            # speed"). Apply ONLY to the inter-segment hop (lift → XY → lower),
            # not the first object's full approach. 0 / unset → defaults.
            _line_z_mm_s = abs(float(getattr(settings, "line_move_z_speed_mm_s", 0.0) or 0.0))
            _line_xy_mm_s = abs(float(getattr(settings, "line_move_xy_speed_mm_s", 0.0) or 0.0))
            _hop_z_fr = (_line_z_mm_s * 60.0) if _line_z_mm_s > 0 else None  # mm/min

            for seg_idx, seg_points in enumerate(segments):
                multi = len(segments) > 1
                seg_label = (well_name if not multi
                             else f"{well_name} obj {seg_idx + 1}/{len(segments)}")

                if seg_idx == 0:
                    # First object in this well: approach from the full travel
                    # height (inter-well / from job start). The MOVE_XY handler
                    # retracts to travel Z and CONFIRMS arrival before XY.
                    commands.append(PrintCommand(
                        type=CommandType.TRAVEL_UP,
                        label=f"Travel up for {seg_label}",
                    ))
                    commands.append(PrintCommand(
                        type=CommandType.MOVE_XY,
                        params={"x": well_x + seg_points[0][0],
                                "y": well_y + seg_points[0][1]},
                        label=f"Move to {seg_label} start",
                    ))
                else:
                    # Subsequent object in the SAME well: a small confirmed hop
                    # (hop_z, ~1 mm) clears the printed material without a costly
                    # full retract. `hop_z` routes MOVE_XY's retract through
                    # ensure_retracted_to(hop_z) — still raise-only / never
                    # descends, and floored by the insert clearance.
                    _hop_params = {"x": well_x + seg_points[0][0],
                                   "y": well_y + seg_points[0][1],
                                   "hop_z": hop_z}
                    if _line_xy_mm_s > 0:
                        _hop_params["xy_speed_mm_s"] = _line_xy_mm_s
                    if _hop_z_fr is not None:
                        # FAST lift-up feedrate for the hop retract.
                        _hop_params["retract_feedrate_mm_min"] = _hop_z_fr
                    commands.append(PrintCommand(
                        type=CommandType.MOVE_XY,
                        params=_hop_params,
                        label=f"Hop to {seg_label} start",
                    ))

                if settings.dwell_after_move > 0:
                    commands.append(PrintCommand(
                        type=CommandType.DWELL,
                        params={"seconds": settings.dwell_after_move},
                        label="Settle",
                    ))

                # Lower to print height. On an inter-segment hop (seg_idx > 0)
                # use the FAST line-move Z feedrate for the descent too.
                _move_z_params = {"z": well_z_height}
                if seg_idx > 0 and _hop_z_fr is not None:
                    _move_z_params["feedrate_mm_min"] = _hop_z_fr
                if getattr(settings, "plate_z_plane_zref_mm", None):
                    # v7.5.x: tell the print-floor clamp WHICH XY this descent is
                    # for, so it can resolve the floor from the tilt plane at this
                    # well instead of a plate-wide scalar. Deterministic — it does
                    # not depend on a cached position read. Emitted only when a
                    # plane is stamped, so a legacy plan is unchanged.
                    _move_z_params["floor_x_mm"] = well_x
                    _move_z_params["floor_y_mm"] = well_y
                commands.append(PrintCommand(
                    type=CommandType.MOVE_Z,
                    params=_move_z_params,
                    label="Lower to print height",
                ))

                # Prime (v7.2: µL amounts, legacy mm fallback)
                prime_uL = settings.get_prime_uL(active_pump)
                prime_mm = settings.get_prime_amount(active_pump)
                if prime_uL > 0:
                    commands.append(PrintCommand(
                        type=CommandType.DISPENSE,
                        params={"pump": active_pump, "amount_uL": prime_uL,
                                "rate_uL_s": settings.get_pump_rate(active_pump)},
                        label=f"Prime {active_pump} ({prime_uL:.2f} µL)",
                    ))
                elif prime_mm > 0:
                    commands.append(PrintCommand(
                        type=CommandType.DISPENSE,
                        params={"pump": active_pump, "amount": prime_mm,
                                "feedrate": settings.pump_feedrate},
                        label=f"Prime {active_pump} (legacy)",
                    ))

                # Print the path (v7.2: include flow_rate_uL_s)
                well_path = [(well_x + px, well_y + py) for px, py in seg_points]
                path_params = {
                    "points": well_path,
                    "pump": active_pump,
                    "flow_rate": flow_rate,
                }
                # If flow_rate looks like µL/s (> 0.05), tag as v7.2
                pump_rate = settings.get_pump_rate(active_pump) if hasattr(settings, 'get_pump_rate') else 0
                if pump_rate > 0:
                    path_params["flow_rate_uL_s"] = pump_rate
                commands.append(PrintCommand(
                    type=CommandType.PRINT_PATH,
                    params=path_params,
                    label=f"Print {seg_label}",
                ))

                # Retract (v7.2: µL amounts, legacy mm fallback)
                retract_uL = settings.get_retract_uL(active_pump) if hasattr(settings, 'get_retract_uL') else 0
                retract_mm = settings.get_retract_amount(active_pump)
                if retract_uL > 0:
                    commands.append(PrintCommand(
                        type=CommandType.DISPENSE,
                        params={"pump": active_pump, "amount_uL": -retract_uL,
                                "rate_uL_s": settings.get_pump_rate(active_pump)},
                        label=f"Retract {active_pump} ({retract_uL:.2f} µL)",
                    ))
                elif retract_mm > 0:
                    commands.append(PrintCommand(
                        type=CommandType.DISPENSE,
                        params={"pump": active_pump, "amount": -retract_mm,
                                "feedrate": settings.pump_feedrate},
                        label=f"Retract {active_pump} (legacy)",
                    ))

    # Final travel up — always retract the needle out of the well to the
    # travel Z when the job ends.
    commands.append(PrintCommand(
        type=CommandType.TRAVEL_UP,
        label="Final travel up",
    ))
    # v7.5.x: optionally return XY to the zero reference (0,0). Quick Print
    # passes return_home=False so the stage stays where it finished printing
    # (needle already retracted by the TRAVEL_UP above) instead of driving the
    # plate all the way back to origin.
    if return_home:
        commands.append(PrintCommand(
            type=CommandType.HOME_XY,
            label="Return home",
        ))

    return PrintJob(
        name=job_name,
        description=f"{len(well_positions)} wells, {settings.num_layers} layers",
        settings=settings,
        commands=commands,
    )


# ═══════════════════════════════════════════════════════════════════
# v7.1: Trajectory Executor (P8.2)
# ═══════════════════════════════════════════════════════════════════

class TrajectoryExecutor:
    """
    P8.2: Executes a time-parameterised trajectory using the MotionController.

    Runs the motion control loop at a fixed timestep, commanding the stage
    to follow the planned trajectory. Records actual vs planned positions
    via the PrintRecorder if available.

    This replaces discrete MOVE_XY + EXTRUDE commands with smooth,
    continuous trajectory tracking.
    """

    def __init__(self, controller, recorder=None, exec_logger=None):
        """
        Args:
            controller: StageController instance
            recorder: Optional PrintRecorder for data logging
            exec_logger: Optional PrintExecutionLogger (v7.5.x JSONL log)
        """
        self.controller = controller
        self.recorder = recorder
        self.exec_logger = exec_logger
        self._abort_flag = threading.Event()

    def execute(
        self,
        waypoints: list,
        pause_event: threading.Event | None = None,
        on_progress: Callable | None = None,
    ) -> bool:
        """
        Execute a trajectory (list of Waypoints).

        Args:
            waypoints: List of Waypoint objects with t, x, y, z, p1, p2, p3
            pause_event: Event that blocks when cleared (for pause support)
            on_progress: Callback(current_idx, total, message)

        Returns:
            True if completed, False if aborted
        """
        if not waypoints:
            logger.warning("TrajectoryExecutor: empty waypoint list")
            return True

        total = len(waypoints)
        ctrl = self.controller
        t_start = time.monotonic()

        logger.info(f"TrajectoryExecutor: starting {total} waypoints, "
                     f"duration={waypoints[-1].t:.2f}s")
        lg = self.exec_logger
        if lg:
            lg.log("traj_start", n_waypoints=total,
                   plan_duration_s=round(float(waypoints[-1].t), 3))

        # v7.2.7: Set stage speed for trajectory
        try:
            _max_spd = 0.0
            for _j in range(1, min(len(waypoints), 100)):
                _dt_wp = waypoints[_j].t - waypoints[_j-1].t
                if _dt_wp > 1e-6:
                    _dx = waypoints[_j].x - waypoints[_j-1].x
                    _dy = waypoints[_j].y - waypoints[_j-1].y
                    _spd = math.sqrt(_dx*_dx + _dy*_dy) / _dt_wp
                    _max_spd = max(_max_spd, _spd)
            if _max_spd > 0 and hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                # v7.2.7: use set_speed_mm_s for trajectory
                _speed_info = ""
                if hasattr(ctrl.xy_stage, "set_speed_mm_s"):
                    ctrl.xy_stage.set_speed_mm_s(_max_spd * 1.5)
                    _speed_info = f"{_max_spd * 1.5:.1f} mm/s"
                else:
                    _sms_val = int(min(_max_spd * 1.5 * 1000.0, 50000))
                    ctrl.xy_stage.set_velocity(_sms_val)
                    _speed_info = f"SMS={_sms_val}"
                logger.info(f"v7.2.7: Trajectory speed {_max_spd:.1f} mm/s, {_speed_info}")
                if lg:
                    lg.log("speed_set", context="trajectory",
                           mm_s=round(_max_spd * 1.5, 3), info=_speed_info)
        except Exception as _e:
            logger.warning(f"v7.2.7: Could not set trajectory speed: {_e}")

        # v7.2.7: Track previous axis values to skip unchanged commands
        _prev_z = None
        _prev_pumps = [None, None, None]

        for i, wp in enumerate(waypoints):
            # Check abort
            if self._abort_flag.is_set():
                logger.info("TrajectoryExecutor: aborted")
                return False

            # Wait if paused
            if pause_event is not None:
                pause_event.wait()
                if self._abort_flag.is_set():
                    return False

            # Wait until waypoint time
            t_target = t_start + wp.t
            t_now = time.monotonic()
            if t_target > t_now:
                time.sleep(t_target - t_now)

            # v7.5.x exec log: lateness vs plan + sampler lag target.
            # Logged sparsely (every 25th wp, or any wp >0.25s late) to
            # keep file size sane on dense trajectories.
            if lg:
                late_s = max(0.0, time.monotonic() - t_target)
                if late_s > 0.25 or i % 25 == 0 or i == total - 1:
                    lg.log("traj_wp", i=i, t_plan=round(float(wp.t), 3),
                           late_s=round(late_s, 3),
                           x_mm=round(float(wp.x), 4),
                           y_mm=round(float(wp.y), 4),
                           z_mm=round(float(wp.z), 4))
                try:
                    zero = ctrl.zero_position
                    lg.note_xy_target(wp.x * 1000.0 + zero["x"],
                                      wp.y * 1000.0 + zero["y"])
                except Exception:
                    pass

            # Command stage position
            # XY: convert mm to stage units (µsteps) — done by controller
            if ctrl.is_xy_connected:
                ctrl.move_xy_absolute(wp.x, wp.y, from_zero_ref=True, fast=False)

            # Z axis
            if ctrl.is_zp_connected:
                # v7.2.7: skip Z if unchanged
                if _prev_z is None or abs(wp.z - _prev_z) > 0.001:
                    ctrl.move_z_absolute(wp.z, from_zero_ref=True)
                    _prev_z = wp.z

            # Pumps (move to absolute plunger position)
            if ctrl.is_zp_connected and ctrl.zp_stage:
                # v7.4.2: honor configurable per-machine axis_map
                _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP)
                for pump_id, wp_val in [("P1", wp.p1), ("P2", wp.p2), ("P3", wp.p3)]:
                    mapped = _axis_map.get(pump_id, AXIS_MAP.get(pump_id))
                    if mapped and wp_val != 0.0:
                        ctrl.zp_stage.move_absolute(
                            {mapped: wp_val + ctrl.zero_position.get(pump_id, 0)},
                            fast=False,
                        )

            # Record to PrintRecorder
            if self.recorder and self.recorder.is_recording:
                actual_xy = ctrl.get_xy_position(cached=True)
                actual_zp = ctrl.get_zp_position(cached=True)
                ax = actual_xy[0] if actual_xy[0] is not None else 0.0
                ay = actual_xy[1] if actual_xy[1] is not None else 0.0
                # v7.4.2 hotfix: pluck each logical axis via axis_map so
                # the CSV columns are in (Z, P1, P2, P3) order regardless
                # of how Marlin's physical tuple is wired.
                az = ctrl.zp_logical_value(actual_zp, "Z") or 0.0
                ap1 = ctrl.zp_logical_value(actual_zp, "P1") or 0.0
                ap2 = ctrl.zp_logical_value(actual_zp, "P2") or 0.0
                ap3 = ctrl.zp_logical_value(actual_zp, "P3") or 0.0

                # Convert actual XY from stage coords to zero-ref coords
                ax -= ctrl.zero_position.get("x", 0)
                ay -= ctrl.zero_position.get("y", 0)
                az -= ctrl.zero_position.get("Z", 0)

                self.recorder.record_sample(
                    t=wp.t,
                    planned=(wp.x, wp.y, wp.z, wp.p1, wp.p2, wp.p3),
                    actual_xy=(ax, ay),
                    actual_zp=(az, ap1, ap2, ap3),
                    segment_id=getattr(wp, "segment_id", 0),
                    is_travel=getattr(wp, "is_travel", False),
                    is_retract=getattr(wp, "is_retract", False),
                )

            # Progress reporting (every 10 waypoints)
            if on_progress and i % 10 == 0:
                on_progress(i, total, f"Trajectory {i}/{total}")

        logger.info("TrajectoryExecutor: trajectory complete")
        if lg:
            lg.log("traj_end",
                   wall_s=round(time.monotonic() - t_start, 3),
                   plan_s=round(float(waypoints[-1].t), 3))
        return True

    def abort(self):
        """Signal the executor to stop."""
        self._abort_flag.set()

    def reset(self):
        """Reset abort flag for reuse."""
        self._abort_flag.clear()


# ═══════════════════════════════════════════════════════════════════
# v7.2.9: Direct Command Executor (blocking hardware moves)
# ═══════════════════════════════════════════════════════════════════

class DirectCommandExecutor:
    """Executes individual hardware moves with completion confirmation.

    Used by HybridPlanExecutor for service steps (travel, waste, wash,
    ink load) where we need to wait for the stage to physically arrive
    before proceeding.
    """

    def __init__(self, controller: "StageController", exec_logger=None):
        self.ctrl = controller
        self.exec_logger = exec_logger
        self._abort = threading.Event()

    def abort(self):
        self._abort.set()

    def move_xy(self, x_mm: float, y_mm: float,
                timeout_s: float = 15.0) -> bool:
        """Move XY to position and wait for arrival."""
        if not self.ctrl.is_xy_connected:
            return True
        lg = self.exec_logger
        t0 = time.monotonic()
        if lg:
            lg.log("xy_cmd", context="blocking_travel",
                   **lg.xy_cmd_fields(self.ctrl, x_mm, y_mm))
        self.ctrl.move_xy_absolute(x_mm, y_mm, from_zero_ref=True, fast=False)
        ok = self.ctrl.wait_for_xy_arrival(
            x_mm, y_mm, tolerance_mm=0.5, timeout_s=timeout_s)
        if lg:
            lg.log("xy_arrival", ok=bool(ok),
                   duration_s=round(time.monotonic() - t0, 3),
                   timeout_s=timeout_s)
        return ok

    def move_z(self, z_mm: float, feedrate_mm_min: float | None = None,
               timeout_s: float = 10.0) -> bool:
        """Move Z to position and wait for arrival."""
        if not self.ctrl.is_zp_connected:
            return True
        lg = self.exec_logger
        t0 = time.monotonic()
        self.ctrl.move_z_absolute(z_mm, from_zero_ref=True,
                                  feedrate_mm_min=feedrate_mm_min)
        ok = self.ctrl.wait_for_z_arrival(
            z_mm, tolerance_mm=0.1, timeout_s=timeout_s)
        if lg:
            lg.log("z_move", context="blocking", z_mm=round(z_mm, 4),
                   feedrate_mm_min=feedrate_mm_min, ok=bool(ok),
                   duration_s=round(time.monotonic() - t0, 3))
        return ok

    def raise_z(self, z_mm: float, feedrate_mm_min: float | None = None,
                timeout_s: float = 15.0) -> bool:
        """Retract Z UP to ``z_mm`` for a lift-OUT-of-print / pre-travel move.

        v7.5.x: prefers the controller's ``ensure_retracted_to`` so the lift's
        first millimetre is GENTLE (slow-then-fast — the deposited bead can't
        peel off with the needle) and raise-only / polarity-safe / insert-
        floored / confirmed. Falls back to a plain confirmed :meth:`move_z` on
        an older controller. Use this for every lift; use :meth:`move_z` for
        DESCENTS (which must not be slowed)."""
        ctrl = self.ctrl
        if not ctrl.is_zp_connected:
            return True
        if hasattr(ctrl, "ensure_retracted_to"):
            try:
                return bool(ctrl.ensure_retracted_to(
                    float(z_mm), timeout_s=timeout_s,
                    feedrate_mm_min=feedrate_mm_min))
            except TypeError:
                return bool(ctrl.ensure_retracted_to(float(z_mm)))
        return self.move_z(z_mm, feedrate_mm_min=feedrate_mm_min,
                           timeout_s=timeout_s)

    def move_pump(self, pump_id: str, volume_uL: float,
                  rate_uL_s: float | None = None) -> bool:
        """Move pump and wait estimated duration."""
        if not self.ctrl.is_zp_connected:
            return True
        if self.exec_logger:
            self.exec_logger.log("extrude", context="service",
                                 pump=pump_id, vol_uL=round(volume_uL, 4),
                                 rate_uL_s=rate_uL_s)
        # v7.5.x: discrete actuation → bracket with the configured pump settle
        # dwell. This method keeps its own abort-aware completion wait below
        # (better than move_pump_uL's blind sleep), so it does NOT pass
        # settle=True; it brackets explicitly instead.
        _settle_s = 0.0
        if hasattr(self.ctrl, "pump_settle_time_s"):
            try:
                _settle_s = max(0.0, float(self.ctrl.pump_settle_time_s()))
            except Exception:
                _settle_s = 0.0
        if _settle_s > 0:
            time.sleep(_settle_s)            # pre-move settle
        self.ctrl.move_pump_uL(pump_id, volume_uL, rate_uL_s)
        # Estimate pump move duration and wait
        rate = rate_uL_s or 5.0
        est_s = abs(volume_uL) / rate + 0.5  # generous margin
        deadline = time.monotonic() + est_s
        while time.monotonic() < deadline:
            if self._abort.is_set():
                return False
            time.sleep(0.1)
        if _settle_s > 0:
            time.sleep(_settle_s)            # post-move settle
        return True

    def dwell(self, seconds: float) -> bool:
        """Hold position for a duration (with abort check)."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._abort.is_set():
                return False
            time.sleep(min(0.1, seconds))
        return True

    def travel_to_well(self, wx: float, wy: float, settings) -> bool:
        """5-phase blocking travel with settle delays between axes.

        Phase 1: Z up to safe height (fast feedrate)
        Phase 2: 1s settle
        Phase 3: XY fast travel (at service_xy_speed)
        Phase 4: 1s settle
        Phase 5: Z down into well (fast above top_z, slow entry below)
        """
        safe_z = settings.travel_z_height
        top_z = getattr(settings, 'top_z_height', 0.0)
        print_z = settings.print_z_height
        z_fast = getattr(settings, 'fast_z_feedrate_mm_min', None)
        z_entry = getattr(settings, 'entry_z_feedrate_mm_min', None)

        # Phase 1: raise to safe Z — gentle slow first mm (lift out of any
        # previous print without peeling the bead), then fast.
        if not self.raise_z(safe_z, feedrate_mm_min=z_fast):
            return False
        self.dwell(1.0)  # settle after Z up

        # v7.2.9: Set XY speed to service speed before travel.
        # Without this, the stage uses whatever SMS was last set
        # (e.g. the slower print speed), causing travel to take
        # much longer than the estimate predicts.
        service_speed = getattr(settings, 'service_xy_speed_mm_s', None)
        if service_speed and self.ctrl.is_xy_connected:
            xy = self.ctrl.xy_stage
            if xy and hasattr(xy, 'set_speed_mm_s'):
                xy.set_speed_mm_s(service_speed)

        # Phase 2: fast XY travel (blocking)
        if not self.move_xy(wx, wy, timeout_s=20.0):
            logger.warning(f"XY travel to ({wx:.1f}, {wy:.1f}) timed out")
        self.dwell(1.0)  # settle after XY travel

        # Phase 3: lower into well
        if top_z > 0:
            approach_z = top_z + 0.5
            if not self.move_z(approach_z, feedrate_mm_min=z_fast):
                return False
            if not self.move_z(print_z, feedrate_mm_min=z_entry):
                return False
        else:
            if not self.move_z(print_z, feedrate_mm_min=z_entry):
                return False

        # Dwell after arrival
        dwell_s = getattr(settings, 'dwell_after_move', 0)
        if dwell_s > 0:
            self.dwell(dwell_s)
        return True

    def raise_from_well(self, settings) -> bool:
        """Raise Z out of the well to safe travel height — gentle slow first mm
        (so the deposited bead doesn't peel off with the needle), then fast."""
        z_fast = getattr(settings, 'fast_z_feedrate_mm_min', None)
        return self.raise_z(settings.travel_z_height, feedrate_mm_min=z_fast)


# ═══════════════════════════════════════════════════════════════════
# v7.2.9: Hybrid Plan Executor
# ═══════════════════════════════════════════════════════════════════

class HybridPlanExecutor:
    """Iterates PlanSteps — service steps as blocking commands, PRINT
    steps as coordinated trajectory playback.

    This replaces the all-in-one trajectory approach where every step
    (travel, waste, wash, ink, print) was pre-compiled into a single
    time-parameterized waypoint list.
    """

    def __init__(self, controller: "StageController",
                 plan, well_model, plate, path_points,
                 settings: PrintSettings, hw_config=None,
                 recorder=None, exec_logger=None):
        self.controller = controller
        self.plan = plan
        self.well_model = well_model
        self.plate = plate
        self.path_points = path_points
        self.settings = settings
        self.hw_config = hw_config
        self.recorder = recorder
        self.exec_logger = exec_logger
        self._abort_flag = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # not paused initially

    def _plate_axis_sign(self) -> tuple[float, float]:
        """v7.5.x: per-machine plate-local→stage axis sign. ``get_well_position``
        returns a PLATE-LOCAL (A1-relative mm) offset; multiply by this before
        using it as a zero-ref/stage coordinate so the needle reaches the
        physically-correct well on a 180°-mounted stage (ME3B V1)."""
        ctrl = self.controller
        if ctrl is not None and hasattr(ctrl, "plate_axis_sign"):
            try:
                sign = ctrl.plate_axis_sign()
                return (float(sign[0]), float(sign[1]))
            except Exception:
                pass
        return (1.0, 1.0)

    def _well_xy_mm(self, well_name) -> tuple[float, float]:
        """v7.5.x: well centre in ZERO-REF mm — the CALIBRATED taught position
        (stamped on ``settings.well_positions_mm``) when available, else the
        GEOMETRIC plate-local offset × ``plate_axis_sign``. This is what makes a
        print land on the physically-taught well instead of a grid anchored at
        the stage origin (which, on ME3B V1's (-1,-1) flip, clamps to 0,0). See
        :func:`PrintTrajectoryPlanner.resolve_well_xy_mm`."""
        from SupportClasses.PrintTrajectoryPlanner import resolve_well_xy_mm
        return resolve_well_xy_mm(well_name, self.plate, self.settings)

    def abort(self):
        self._abort_flag.set()

    # ── Time estimation ───────────────────────────────────────────

    def estimate_time(self) -> float:
        """Estimate total execution time in seconds.

        Walks every plan step and sums:
        - XY travel time (distance / speed)
        - Z travel time (distance / feedrate)
        - Pump time (volume / rate)
        - Fixed dwells (settle, wash, etc.)
        - Print trajectory time (path length / print speed)
        """
        try:
            from SupportClasses.PrintPlanOfAction import PlanStepType
        except ImportError:
            return 0.0

        steps = getattr(self.plan, 'steps', [])
        if not steps:
            return 0.0

        s = self.settings
        safe_z = s.travel_z_height
        print_z = s.print_z_height
        top_z = getattr(s, 'top_z_height', 0.0)
        z_fast = getattr(s, 'fast_z_feedrate_mm_min', s.z_feedrate) / 60.0
        z_entry = getattr(s, 'entry_z_feedrate_mm_min', s.z_feedrate) / 60.0
        xy_travel = getattr(s, 'service_xy_speed_mm_s', 10.0)
        xy_print = getattr(s, 'print_speed_mm_s', 5.0)
        pump_rate = getattr(s, 'service_pump_rate_uL_s', 5.0)

        # Precompute print path length once
        path_len = 0.0
        pts = self.path_points
        if pts and len(pts) >= 2:
            for i in range(1, len(pts)):
                dx = pts[i][0] - pts[i - 1][0]
                dy = pts[i][1] - pts[i - 1][1]
                path_len += math.sqrt(dx * dx + dy * dy)

        # Track current position for distance calculations
        cur_x, cur_y, cur_z = 0.0, 0.0, safe_z
        total = 0.0
        SETTLE = 1.0  # 1s settle between axis changes

        def _z_travel(from_z, to_z):
            """Time for a Z move.

            Descending: fast above top_z, slow entry below.
            Ascending: always fast (raise_from_well uses z_fast).
            """
            dist = abs(to_z - from_z)
            if dist < 0.01:
                return 0.0
            # Ascending: always fast
            if to_z > from_z:
                return dist / max(z_fast, 0.1)
            # Descending: split at top_z boundary
            if top_z > 0 and from_z > top_z:
                fast_dist = max(0, from_z - top_z)
                slow_dist = max(0, top_z - to_z)
                return fast_dist / max(z_fast, 0.1) + slow_dist / max(z_entry, 0.1)
            # Descending but already below top_z: all slow
            if top_z > 0 and from_z <= top_z:
                return dist / max(z_entry, 0.1)
            return dist / max(z_fast, 0.1)

        def _xy_travel(x1, y1, x2, y2, speed):
            d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return d / max(speed, 0.1) if d > 0.01 else 0.0

        def _service_time(role, step):
            """Time for the action inside a well (not counting travel)."""
            if role == "waste":
                vol = getattr(step, 'volume_uL', 50.0) or 50.0
                return vol / max(pump_rate, 0.1) + 0.5
            elif role == "wash":
                return 5.0
            elif role == "buffer":
                vol = getattr(step, 'volume_uL', 5.0) or 5.0
                return vol / max(pump_rate, 0.1) + 1.0
            elif role == "ink":
                vol = getattr(step, 'volume_uL', 50.0) or 50.0
                return vol / max(pump_rate, 0.1) + 1.0
            return 0.0

        def _travel_to_well_time(wx, wy):
            """Time for full travel_to_well: Z up, settle, XY, settle, Z down."""
            nonlocal cur_x, cur_y, cur_z
            t = 0.0
            # Z up
            t += _z_travel(cur_z, safe_z) + SETTLE
            # XY travel
            t += _xy_travel(cur_x, cur_y, wx, wy, xy_travel) + SETTLE
            # Z down
            t += _z_travel(safe_z, print_z)
            t += getattr(s, 'dwell_after_move', 0)
            cur_x, cur_y, cur_z = wx, wy, print_z
            return t

        def _raise_time():
            nonlocal cur_z
            t = _z_travel(cur_z, safe_z)
            cur_z = safe_z
            return t

        for step in steps:
            stype = getattr(step, 'step_type', None)
            if stype is None:
                continue

            if stype == PlanStepType.PRINT:
                target_wells = getattr(step, 'target_wells', [])
                for wn in target_wells:
                    try:
                        wx, wy = self._well_xy_mm(wn)
                    except Exception:
                        continue
                    # First path point offset
                    fx = wx + (pts[0][0] if pts else 0.0)
                    fy = wy + (pts[0][1] if pts else 0.0)
                    # Z up + settle + XY to first point + settle + Z down
                    total += _z_travel(cur_z, safe_z) + SETTLE
                    total += _xy_travel(cur_x, cur_y, fx, fy, xy_travel) + SETTLE
                    total += _z_travel(safe_z, print_z)
                    total += getattr(s, 'dwell_after_move', 0)
                    # v7.2.9: Estimate trajectory time accounting for
                    # per-waypoint blocking.  TrajectoryExecutor sends a
                    # blocking move_xy_absolute per waypoint — each blocks
                    # until the stage settles.  When blocking time exceeds
                    # the inter-waypoint interval, the executor falls behind
                    # and total time = sum(blocking times), not the planned
                    # trajectory duration.
                    #
                    # Blocking time per waypoint ≈ move_time + settle_overhead
                    #   move_time = segment_distance / effective_speed
                    #   settle_overhead ≈ 0.1s (PD controller exponential decay)
                    #   effective_speed = max_waypoint_speed * 1.5
                    #     (TrajectoryExecutor sets SMS to 1.5× max segment speed)
                    SETTLE_OVERHEAD_S = 0.25  # per-waypoint blocking settle+cmd
                    traj_time = 0.0
                    try:
                        from SupportClasses.PrintTrajectoryPlanner import (
                            PrintTrajectoryPlanner)
                        _planner = PrintTrajectoryPlanner()
                        _result = _planner.generate_inwell_print(
                            well_x=wx, well_y=wy,
                            pump_id=getattr(step, 'pump_id', 'P1') or 'P1',
                            path_points=pts,
                            settings=s,
                            well_name=wn,
                        )
                        if _result.valid and _result.waypoints:
                            wps = _result.waypoints
                            # Compute max segment speed (same as executor)
                            _max_spd = 0.0
                            for _j in range(1, min(len(wps), 100)):
                                _dt_wp = wps[_j].t - wps[_j-1].t
                                if _dt_wp > 1e-6:
                                    _dx = wps[_j].x - wps[_j-1].x
                                    _dy = wps[_j].y - wps[_j-1].y
                                    _spd = math.sqrt(_dx*_dx + _dy*_dy) / _dt_wp
                                    _max_spd = max(_max_spd, _spd)
                            eff_speed = max(_max_spd * 1.5, 0.1)
                            # Sum per-waypoint blocking times
                            block_total = 0.0
                            for _j in range(1, len(wps)):
                                _dx = wps[_j].x - wps[_j-1].x
                                _dy = wps[_j].y - wps[_j-1].y
                                seg_d = math.sqrt(_dx*_dx + _dy*_dy)
                                dt_plan = wps[_j].t - wps[_j-1].t
                                # Blocking time = move + settle, but at
                                # least the planned interval (sleep covers
                                # idle time if move is fast enough)
                                t_block = seg_d / eff_speed + SETTLE_OVERHEAD_S
                                block_total += max(t_block, dt_plan)
                            traj_time = block_total
                    except Exception:
                        pass
                    # Fallback to naive formula if planner unavailable
                    if traj_time <= 0 and xy_print > 0 and path_len > 0:
                        traj_time = (path_len / xy_print) * s.num_layers
                    total += traj_time
                    # Raise + settle
                    total += SETTLE + _z_travel(print_z, safe_z)
                    cur_x, cur_y, cur_z = fx, fy, safe_z

            elif stype in (PlanStepType.WASTE, PlanStepType.WASH,
                           PlanStepType.REFILL_BUFFER,
                           PlanStepType.LOAD_INK, PlanStepType.GATHER_INK):
                role_map = {
                    PlanStepType.WASTE: "waste", PlanStepType.WASH: "wash",
                    PlanStepType.REFILL_BUFFER: "buffer",
                    PlanStepType.LOAD_INK: "ink",
                    PlanStepType.GATHER_INK: "ink",
                }
                role = role_map.get(stype, "waste")
                well = self._find_well(role)
                if well:
                    wx, wy = self._well_xy_mm(well[0])
                    total += _travel_to_well_time(wx, wy)
                    total += _service_time(role, step)
                    total += _raise_time()

            elif stype == PlanStepType.MOVE_SAFE_Z:
                total += _z_travel(cur_z, safe_z)
                cur_z = safe_z

            elif stype == PlanStepType.TRAVEL_XY:
                tw = getattr(step, 'target_wells', [])
                if tw:
                    try:
                        wx, wy = self._well_xy_mm(tw[0])
                        total += _xy_travel(cur_x, cur_y, wx, wy, xy_travel)
                        cur_x, cur_y = wx, wy
                    except Exception:
                        pass

            elif stype == PlanStepType.INK_SWAP:
                sub_steps = getattr(step, 'sub_steps', [])
                for sub in sub_steps:
                    role = sub if isinstance(sub, str) else "waste"
                    well = self._find_well(role)
                    if well:
                        wx, wy = self._well_xy_mm(well[0])
                        total += _travel_to_well_time(wx, wy)
                        total += _service_time(role, step)
                        total += _raise_time()

            elif stype == PlanStepType.FINAL_CLEANUP:
                sub_steps = getattr(step, 'sub_steps', [])
                for sub in ("waste", "wash"):
                    if sub in sub_steps:
                        well = self._find_well(sub)
                        if well:
                            wx, wy = self._well_xy_mm(well[0])
                            total += _travel_to_well_time(wx, wy)
                            total += _service_time(sub, step)
                            total += _raise_time()

            elif stype == PlanStepType.RETURN_HOME:
                total += _z_travel(cur_z, safe_z) + SETTLE
                total += _xy_travel(cur_x, cur_y, 0, 0, xy_travel)
                cur_x, cur_y, cur_z = 0.0, 0.0, safe_z

        return total

    def execute(self, pause_event: threading.Event | None = None,
                on_progress=None) -> bool:
        """Execute the plan step by step.

        Returns True if completed, False if aborted.
        """
        if pause_event is not None:
            self._pause_event = pause_event

        try:
            from SupportClasses.PrintPlanOfAction import PlanStepType
        except ImportError:
            logger.error("HybridPlanExecutor: PrintPlanOfAction not available")
            return False

        steps = getattr(self.plan, 'steps', [])
        if not steps:
            logger.warning("HybridPlanExecutor: no steps in plan")
            return True

        direct = DirectCommandExecutor(self.controller,
                                       exec_logger=self.exec_logger)
        total_steps = len(steps)

        logger.info(f"HybridPlanExecutor: starting {total_steps} plan steps")

        for step_idx, step in enumerate(steps):
            # Check abort
            if self._abort_flag.is_set():
                logger.info("HybridPlanExecutor: aborted")
                direct.abort()
                return False

            # Check pause
            self._pause_event.wait()
            if self._abort_flag.is_set():
                return False

            stype = getattr(step, 'step_type', None)
            pump = (getattr(step, 'pump_id', None)
                    or getattr(self.settings, 'active_pump', 'P1') or 'P1')

            step_name = stype.name if stype else "UNKNOWN"
            logger.info(f"HybridPlanExecutor: step {step_idx+1}/{total_steps}"
                        f" — {step_name}")
            if self.exec_logger:
                self.exec_logger.log("plan_step", i=step_idx + 1,
                                     total=total_steps, type=step_name,
                                     pump=pump)
            if on_progress:
                on_progress(step_idx, total_steps,
                            f"Step {step_idx+1}/{total_steps}: {step_name}")

            # ── PRINT step: use trajectory ──────────────────────────
            if stype == PlanStepType.PRINT:
                self._execute_print_step(step, pump, pause_event, on_progress)

            # ── Service steps: blocking commands ────────────────────
            elif stype == PlanStepType.WASTE:
                self._execute_service(direct, step, pump, "waste")

            elif stype == PlanStepType.WASH:
                self._execute_service(direct, step, pump, "wash")

            elif stype == PlanStepType.REFILL_BUFFER:
                self._execute_service(direct, step, pump, "buffer")

            elif stype in (PlanStepType.LOAD_INK, PlanStepType.GATHER_INK):
                self._execute_service(direct, step, pump, "ink")

            elif stype == PlanStepType.MOVE_SAFE_Z:
                z_fast = getattr(self.settings, 'fast_z_feedrate_mm_min', None)
                direct.raise_z(self.settings.travel_z_height,
                               feedrate_mm_min=z_fast)

            elif stype == PlanStepType.TRAVEL_XY:
                target_wells = getattr(step, 'target_wells', [])
                if target_wells:
                    try:
                        # v7.5.x CRITICAL SAFETY: TRAVEL_XY is cross-well travel.
                        # Retract to the travel Z and WAIT before the XY move — do
                        # not rely solely on a preceding MOVE_SAFE_Z plan step
                        # (mirrors RETURN_HOME below). v7.5.x: gentle slow first
                        # mm (raise_z) so the bead doesn't peel off the needle.
                        z_fast = getattr(self.settings, 'fast_z_feedrate_mm_min', None)
                        direct.raise_z(self.settings.travel_z_height,
                                       feedrate_mm_min=z_fast)
                        # v7.2.9: Set service speed for XY travel
                        _svc_spd = getattr(self.settings, 'service_xy_speed_mm_s', None)
                        if _svc_spd and self.controller.is_xy_connected:
                            _xy = self.controller.xy_stage
                            if _xy and hasattr(_xy, 'set_speed_mm_s'):
                                _xy.set_speed_mm_s(_svc_spd)
                        wx, wy = self._well_xy_mm(target_wells[0])
                        direct.move_xy(wx, wy, timeout_s=20.0)
                    except Exception as e:
                        logger.warning(f"TRAVEL_XY failed: {e}")

            elif stype == PlanStepType.INK_SWAP:
                sub_steps = getattr(step, 'sub_steps', [])
                for sub in sub_steps:
                    if self._abort_flag.is_set():
                        return False
                    if sub == "waste":
                        self._execute_service(direct, step, pump, "waste")
                    elif sub == "wash":
                        self._execute_service(direct, step, pump, "wash")
                    elif sub == "buffer":
                        self._execute_service(direct, step, pump, "buffer")
                    elif sub == "ink":
                        self._execute_service(direct, step, pump, "ink")

            elif stype == PlanStepType.FINAL_CLEANUP:
                sub_steps = getattr(step, 'sub_steps', [])
                if "waste" in sub_steps:
                    self._execute_service(direct, step, pump, "waste")
                if "wash" in sub_steps:
                    self._execute_service(direct, step, pump, "wash")

            elif stype == PlanStepType.RETURN_HOME:
                z_fast = getattr(self.settings, 'fast_z_feedrate_mm_min', None)
                direct.raise_z(self.settings.travel_z_height,
                               feedrate_mm_min=z_fast)
                # v7.2.9: Set service speed for return travel
                _svc_spd = getattr(self.settings, 'service_xy_speed_mm_s', None)
                if _svc_spd and self.controller.is_xy_connected:
                    _xy = self.controller.xy_stage
                    if _xy and hasattr(_xy, 'set_speed_mm_s'):
                        _xy.set_speed_mm_s(_svc_spd)
                direct.move_xy(0, 0, timeout_s=20.0)

        logger.info("HybridPlanExecutor: all steps complete")
        if on_progress:
            on_progress(total_steps, total_steps, "Complete!")
        return True

    # ── Internal helpers ──────────────────────────────────────────

    def _find_well(self, role: str):
        """Find first well with given role. Returns (name, x, y) or None."""
        from SupportClasses.PrintTrajectoryPlanner import _find_well
        return _find_well(self.well_model, self.plate, role)

    def _execute_service(self, direct: DirectCommandExecutor,
                         step, pump_id: str, role: str):
        """Execute a service step: travel to well → action → raise."""
        well = self._find_well(role)
        if not well:
            logger.info(f"Skipping service '{role}': no well assigned")
            return

        name = well[0]
        # v7.5.x: resolve the role-matched well to its CALIBRATED zero-ref-mm
        # centre (else geometric × sign). _find_well returns the RAW geometric
        # offset, which on ME3B V1 lands service moves on the wrong well.
        wx, wy = self._well_xy_mm(name)

        # Travel to well and lower
        direct.travel_to_well(wx, wy, self.settings)

        # Action depends on role
        if role == "waste":
            vol = getattr(step, 'volume_uL', 50.0) or 50.0
            direct.move_pump(pump_id, vol)  # dispense (positive = push out)
            direct.dwell(0.5)

        elif role == "wash":
            direct.dwell(5.0)

        elif role == "buffer":
            vol = getattr(step, 'volume_uL', 5.0) or 5.0
            direct.move_pump(pump_id, -vol)  # aspirate (negative)
            direct.dwell(1.0)

        elif role == "ink":
            vol = getattr(step, 'volume_uL', 50.0) or 50.0
            direct.move_pump(pump_id, -vol)  # aspirate
            direct.dwell(1.0)

        # Raise from well
        direct.raise_from_well(self.settings)

    def _execute_print_step(self, step, pump_id: str,
                            pause_event, on_progress):
        """Execute a PRINT step: for each well, use blocking moves for
        travel/Z, then trajectory playback for in-well printing only.

        Sequence per well:
          1. Z up to safe height
          2. XY travel to first path point (well_center + path_offset)
          3. Z down to print height
          4. TrajectoryExecutor: in-well print (prime → XY+pump → retract)
          5. Z up to safe height
        """
        target_wells = getattr(step, 'target_wells', [])
        if not target_wells:
            logger.warning("PRINT step with no target wells")
            return

        direct = DirectCommandExecutor(self.controller,
                                       exec_logger=self.exec_logger)
        safe_z = self.settings.travel_z_height
        print_z = self.settings.print_z_height
        top_z = getattr(self.settings, 'top_z_height', 0.0)
        z_fast = getattr(self.settings, 'fast_z_feedrate_mm_min', None)
        z_entry = getattr(self.settings, 'entry_z_feedrate_mm_min', None)

        try:
            from SupportClasses.PrintTrajectoryPlanner import (
                PrintTrajectoryPlanner)

            for well_name in target_wells:
                if self._abort_flag.is_set():
                    return

                try:
                    # v7.5.x: CALIBRATED taught well centre (zero-ref mm) when
                    # available, else the geometric offset × plate_axis_sign.
                    # move_xy below treats these as zero-ref mm. See
                    # resolve_well_xy_mm — this is what lands the print on the
                    # physically-taught well rather than a stage-origin grid.
                    wx, wy = self._well_xy_mm(well_name)
                except Exception:
                    logger.error(f"Well {well_name}: position lookup failed")
                    continue

                # Compute first path point in absolute coords
                first_x = wx + (self.path_points[0][0] if self.path_points else 0.0)
                first_y = wy + (self.path_points[0][1] if self.path_points else 0.0)

                logger.info(f"PRINT: well {well_name} — "
                            f"first point ({first_x:.1f}, {first_y:.1f})")

                # 1. Z up to safe height (fast)
                direct.move_z(safe_z, feedrate_mm_min=z_fast)
                direct.dwell(1.0)

                # 2. XY travel to first path point (at service speed)
                _svc_spd = getattr(self.settings, 'service_xy_speed_mm_s', None)
                if _svc_spd and self.controller.is_xy_connected:
                    _xy = self.controller.xy_stage
                    if _xy and hasattr(_xy, 'set_speed_mm_s'):
                        _xy.set_speed_mm_s(_svc_spd)
                if not direct.move_xy(first_x, first_y, timeout_s=20.0):
                    logger.warning(f"XY travel to first point timed out")
                direct.dwell(1.0)

                # 3. Z down to print height (fast above top_z, slow entry)
                if top_z > 0:
                    direct.move_z(top_z + 0.5, feedrate_mm_min=z_fast)
                    direct.dwell(1.0)
                direct.move_z(print_z, feedrate_mm_min=z_entry)

                # Dwell after arrival
                dwell_s = getattr(self.settings, 'dwell_after_move', 0)
                if dwell_s > 0:
                    direct.dwell(dwell_s)

                # 4. Generate in-well print trajectory (XY + pump only, no Z travel)
                planner = PrintTrajectoryPlanner()
                result = planner.generate_inwell_print(
                    well_x=wx, well_y=wy,
                    pump_id=pump_id,
                    path_points=self.path_points,
                    settings=self.settings,
                    well_name=well_name,
                )

                if not result.valid:
                    logger.error(f"In-well trajectory invalid for "
                                 f"{well_name}: {result.issues}")
                    direct.move_z(safe_z)
                    continue

                logger.info(
                    f"PRINT {well_name}: {len(result.waypoints)} waypoints, "
                    f"{result.total_duration_s:.1f}s")

                # 5. Play back the in-well trajectory
                tex = TrajectoryExecutor(
                    self.controller, recorder=self.recorder,
                    exec_logger=self.exec_logger)
                tex.execute(
                    waypoints=result.waypoints,
                    pause_event=pause_event,
                    on_progress=on_progress,
                )

                # 6. Raise from well (fast)
                direct.dwell(1.0)
                direct.move_z(safe_z, feedrate_mm_min=z_fast)

        except Exception as e:
            logger.error(f"Print step execution failed: {e}", exc_info=True)


# ═══════════════════════════════════════════════════════════════════
# v7.1: Service Sequence Executor (P8.6)
# ═══════════════════════════════════════════════════════════════════

class ServiceSequenceExecutor:
    """
    P8.6: Executes service sequences (waste→wash→buffer→ink) between prints.

    A service sequence automates fluid handling when switching inks or
    refreshing the needle. Steps reference well roles, and the executor
    finds the nearest well with that role to execute the operation.

    Sequence steps: "waste", "wash", "buffer", "ink"
    """

    def __init__(self, controller, well_setup=None):
        """
        Args:
            controller: StageController instance
            well_setup: WellSetupModel with well assignments and roles
        """
        self.controller = controller
        self.well_setup = well_setup

    def execute_sequence(
        self,
        sequence_steps: list[str],
        pump: str,
        fluid_tracker: "FluidColumnTracker | None" = None,
        settings: PrintSettings | None = None,
        on_progress: Callable | None = None,
    ) -> bool:
        """
        Execute a service sequence.

        Args:
            sequence_steps: List of step names ["waste", "wash", "buffer", "ink"]
            pump: Which pump to service (e.g. "P1")
            fluid_tracker: Optional FluidColumnTracker for volume tracking
            settings: Print settings for Z heights and feedrates
            on_progress: Callback(step_name, step_idx, total_steps)

        Returns:
            True if completed successfully
        """
        if not self.well_setup or not sequence_steps:
            logger.debug("ServiceSequence: no setup or empty sequence, skipping")
            return True

        if settings is None:
            settings = PrintSettings()

        ctrl = self.controller
        total = len(sequence_steps)

        for idx, step_name in enumerate(sequence_steps):
            if on_progress:
                on_progress(step_name, idx, total)

            logger.info(f"ServiceSequence: step {idx+1}/{total} — {step_name} ({pump})")

            if step_name == "waste":
                self._do_waste(ctrl, pump, fluid_tracker, settings)
            elif step_name == "wash":
                self._do_wash(ctrl, pump, settings)
            elif step_name == "buffer":
                self._do_buffer(ctrl, pump, fluid_tracker, settings)
            elif step_name == "ink":
                self._do_ink_pickup(ctrl, pump, fluid_tracker, settings)
            else:
                logger.warning(f"ServiceSequence: unknown step '{step_name}'")

        logger.info(f"ServiceSequence: completed {total} steps for {pump}")
        return True

    def _do_waste(self, ctrl, pump, tracker, settings):
        """Dispense old ink + contaminated buffer into waste well."""
        # Travel to waste well position (would come from well_setup)
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.5)
        # Dispense: push the pump to expel the contents out
        dispense_amount = 2.0  # mm (configurable from behavior)
        ctrl.move_pump_relative(pump, dispense_amount, settings.pump_feedrate)
        time.sleep(1.0)
        if tracker:
            tracker.record_waste(pump)

    def _do_wash(self, ctrl, pump, settings):
        """Jiggle needle in wash well to clean exterior."""
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.3)
        # Small Z oscillation to agitate wash fluid
        for _ in range(3):
            ctrl.move_z_relative(-1.0, settings.z_feedrate)
            time.sleep(0.2)
            ctrl.move_z_relative(1.0, settings.z_feedrate)
            time.sleep(0.2)

    def _do_buffer(self, ctrl, pump, tracker, settings):
        """Aspirate fresh buffer."""
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.3)
        aspirate_amount = -1.0  # mm (negative = aspirate)
        ctrl.move_pump_relative(pump, aspirate_amount, settings.pump_feedrate)
        time.sleep(1.0)
        if tracker:
            tracker.record_buffer(pump, abs(aspirate_amount))

    def _do_ink_pickup(self, ctrl, pump, tracker, settings):
        """Aspirate ink from ink well."""
        ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
        time.sleep(0.3)
        aspirate_amount = -0.5  # mm (configurable from behavior)
        ctrl.move_pump_relative(pump, aspirate_amount, settings.pump_feedrate)
        time.sleep(1.5)
        if tracker:
            tracker.record_ink_pickup(pump, abs(aspirate_amount))


# ═══════════════════════════════════════════════════════════════════
# v7.1: Fluid Column Tracker (P8.7, P8.8, P8.9)
# ═══════════════════════════════════════════════════════════════════

class FluidColumnTracker:
    """
    P8.7: Tracks fluid column state for each pump during printing.
    P8.8: Detects when ink change is needed and triggers service sequences.
    P8.9: Handles incremental vs continuous printing modes.

    Works alongside the FluidColumn dataclass from PhysicalModels, but
    operates at a higher level — tracking print-time state changes and
    deciding when service sequences are needed.
    """

    def __init__(self):
        # Per-pump state
        self._columns: dict[str, dict] = {
            "P1": {"ink_name": None, "ink_remaining_uL": 0, "mode": "incremental"},
            "P2": {"ink_name": None, "ink_remaining_uL": 0, "mode": "incremental"},
            "P3": {"ink_name": None, "ink_remaining_uL": 0, "mode": "incremental"},
        }
        self._refill_threshold_uL = 5.0

    def configure_pump(
        self,
        pump: str,
        ink_name: str | None,
        initial_volume_uL: float = 0.0,
        mode: str = "incremental",
    ) -> None:
        """Configure a pump's initial state."""
        self._columns[pump] = {
            "ink_name": ink_name,
            "ink_remaining_uL": initial_volume_uL,
            "mode": mode,
        }

    def needs_ink_change(self, pump: str, required_ink: str | None) -> bool:
        """
        P8.8: Check if this pump needs an ink change before printing.

        Returns True if the pump's current ink differs from required_ink.
        """
        if required_ink is None:
            return False
        current = self._columns.get(pump, {}).get("ink_name")
        return current is not None and current != required_ink

    def needs_refill(self, pump: str) -> bool:
        """Check if pump ink volume is below refill threshold."""
        col = self._columns.get(pump, {})
        if col.get("mode") == "continuous":
            return col.get("ink_remaining_uL", 0) < self._refill_threshold_uL
        # Incremental mode always picks up before each print
        return True

    def is_continuous_mode(self, pump: str) -> bool:
        """P8.9: Check if pump is in continuous mode."""
        return self._columns.get(pump, {}).get("mode") == "continuous"

    def record_dispense(self, pump: str, volume_uL: float) -> None:
        """Record that a volume was dispensed (printing)."""
        col = self._columns.get(pump, {})
        col["ink_remaining_uL"] = max(0, col.get("ink_remaining_uL", 0) - volume_uL)

    def record_waste(self, pump: str) -> None:
        """Record waste dispense — ink column is now empty."""
        col = self._columns.get(pump, {})
        col["ink_remaining_uL"] = 0

    def record_buffer(self, pump: str, volume_uL: float) -> None:
        """Record buffer aspiration."""
        pass  # Buffer tracked separately in FluidColumn if needed

    def record_ink_pickup(self, pump: str, volume_uL: float) -> None:
        """Record ink aspiration."""
        col = self._columns.get(pump, {})
        col["ink_remaining_uL"] = col.get("ink_remaining_uL", 0) + volume_uL

    def set_ink(self, pump: str, ink_name: str) -> None:
        """Update the ink currently loaded in a pump."""
        if pump in self._columns:
            self._columns[pump]["ink_name"] = ink_name

    def get_state(self) -> dict:
        """Get serialisable state for persistence."""
        return dict(self._columns)

    def restore_state(self, state: dict) -> None:
        """Restore from saved state."""
        for pump in ("P1", "P2", "P3"):
            if pump in state:
                self._columns[pump] = dict(state[pump])


# ═══════════════════════════════════════════════════════════════════
# Print Executor
# ═══════════════════════════════════════════════════════════════════

class PrintManager:
    """
    Executes print jobs sequentially with pause/resume/abort support.
    
    Runs print execution in a dedicated thread. Reports progress via
    the on_progress callback and state changes via on_state_changed.
    
    Session 4 additions:
    - SWITCH_PUMP command execution (Task 1)
    - Position logging at key events (Task 2)

    v7.1 additions:
    - TRAJECTORY command via TrajectoryExecutor (P8.2, P8.3)
    - PrintRecorder auto-start/stop (P8.4)
    - WorkspaceConfig on PrintJob (P8.5)
    - SERVICE_SEQUENCE command via ServiceSequenceExecutor (P8.6)
    - FluidColumnTracker for ink state management (P8.7, P8.8)
    - Incremental vs continuous mode handling (P8.9)
    """

    # v7.5.x: drain Marlin's planner buffer (M400) every N PRINT_PATH segments
    # so a long open-loop path can never admit pump G0s faster than the board
    # executes them and saturate the buffer. 8 ≈ half Marlin's default 16-block
    # buffer, so depth stays comfortably bounded.
    _PATH_BARRIER_EVERY = 8

    # v7.5.x (Finding C): minimum per-emission pump volume (µL). A single print
    # segment's dispense can round below this on finely-sampled paths / thin
    # beads, but the volume is REAL — instead of dropping it (was: silent under-
    # extrusion, worst at low extrusion where the operator then over-cranks the
    # modifier to compensate), _execute_print_path ACCUMULATES the residual and
    # emits one pump move once the pending total crosses this floor, flushing
    # the tail at path end. So the total dispensed volume is conserved
    # regardless of segment sampling.
    _PATH_PUMP_EMIT_MIN_UL = 0.001

    # v7.5.x (confirmed per-segment printing): position tolerance (µm) for the
    # per-segment XY arrival wait when ``settings.confirm_each_segment`` is set.
    # A segment is considered "arrived" once the stage is within this of the
    # commanded point. Loose enough that the Prior's settle jitter doesn't burn
    # the timeout, tight relative to a bead (needle IDs are hundreds of µm).
    _SEGMENT_SETTLE_TOL_UM = 40

    # v7.5.x: velocity-following control-loop tuning (see PrintSettings.
    # velocity_follow / _execute_print_path_velocity).
    _VEL_CONTROL_HZ = 25.0        # control-loop rate (poll + re-command velocity)
    _VEL_LOOKAHEAD_MM = 0.6       # pure-pursuit carrot distance ahead of progress
    _VEL_DECEL_MM = 1.5           # ramp speed down over the last this-much of path
    _VEL_ARRIVE_TOL_UM = 40       # "arrived at the end" position tolerance
    _VEL_STALE_S = 1.0            # no fresh position for this long → stop (safety)
    _VEL_MAX_CROSS_TRACK_MM = 3.0  # perpendicular error over this = runaway → abort
    _VEL_RUNAWAY_TICKS = 8        # consecutive over-threshold ticks before aborting
    # Cap how far the arc-length projection may advance per tick to the
    # physically-plausible distance (print_speed × real dt × this). This stops
    # the projection SNAPPING to a nearby-but-later loop of a spiral / self-
    # intersecting path (which teleports the carrot → runaway). It stays far
    # below one loop's circumference (loops are a full revolution apart in arc
    # length) yet well above the true per-tick advance.
    _VEL_SNAP_GUARD = 6.0
    _VEL_MAX_SNAP_MM = 1.5        # absolute ceiling on per-tick s advance (mm)
    # v7.5.x: corner-aware speed scheduling defaults (used when settings don't
    # override). corner_angle = turn angle counted as a corner; corner_speed_
    # factor = fraction of print speed allowed at the sharpest (180°) corner.
    _VEL_CORNER_ANGLE_DEG = 30.0
    _VEL_CORNER_SPEED_FACTOR = 0.4

    def __init__(self, controller):
        """
        Args:
            controller: StageController instance with connected stages
        """
        self.controller = controller
        self.job: Optional[PrintJob] = None
        self.state = PrintState.IDLE

        # Active pump tracking (Session 4: multi-material)
        self._active_pump = "P1"

        # Progress callback: (current_step: int, total_steps: int, message: str)
        self.on_progress: Optional[Callable] = None
        # State change callback: (new_state: PrintState)
        self.on_state_changed: Optional[Callable] = None
        # v7.7: per-sample telemetry from the velocity/feed-plan follower, at the
        # same ~5 Hz decimation the exec log already used (so the ~25 Hz control
        # loop gains no work). Receives the `vel_sample` dict. MUST NOT BLOCK —
        # the GUI wires this to a queued Qt signal and coalesces on its own
        # timer. Exceptions are swallowed so a display bug can never stop a print.
        self.on_vel_sample: Optional[Callable] = None

        # Thread control
        self._thread: Optional[threading.Thread] = None
        self._pause_event = threading.Event()
        self._pause_event.set()  # Not paused initially
        self._abort_flag = threading.Event()

        # Current step tracking
        self._current_step = 0

        # Enhancement 3: Print resume support
        self._resume_interval = 20  # Save progress every N commands
        self._start_time: Optional[float] = None

        # Enhancement 6: Print history (set externally by GUI/main)
        self.print_history = None

        # v7.1 P8.4: PrintRecorder (set externally or created on start)
        self.recorder: Optional[object] = None  # PrintRecorder instance

        # v7.5.x: machine-readable JSONL execution log (auto-created on
        # start() unless one was injected; see PrintExecutionLogger).
        self.exec_logger: Optional[PrintExecutionLogger] = None

        # v7.1 P8.2: TrajectoryExecutor
        self._trajectory_executor: Optional[TrajectoryExecutor] = None

        # v7.1 P8.6: Service sequence executor
        self._service_executor: Optional[ServiceSequenceExecutor] = None

        # v7.1 P8.7: Fluid column tracking
        self.fluid_tracker = FluidColumnTracker()

    # ── Job Management ─────────────────────────────────────────────

    def load_file(self, filepath: str):
        """Load a print job from a file."""
        if self.state == PrintState.RUNNING:
            raise RuntimeError("Cannot load file while printing")
        self.job = load_print_file(filepath)
        self._current_step = 0
        self._active_pump = "P1"
        self._set_state(PrintState.IDLE)
        logger.info(f"Loaded print job: {self.job.name} ({self.job.total_steps} commands)")

    def load_job(self, job: PrintJob):
        """Load a pre-built PrintJob directly."""
        if self.state == PrintState.RUNNING:
            raise RuntimeError("Cannot load job while printing")
        self.job = job
        self._current_step = 0
        self._active_pump = "P1"
        self._set_state(PrintState.IDLE)
        logger.info(f"Loaded print job: {self.job.name} ({self.job.total_steps} commands)")

    # ── Execution Control ──────────────────────────────────────────

    def start(self):
        """Start executing the loaded print job."""
        if self.job is None:
            raise RuntimeError("No print job loaded")
        if self.state == PrintState.RUNNING:
            logger.warning("Print already running")
            return

        self._abort_flag.clear()
        self._pause_event.set()
        self._current_step = 0
        self._active_pump = "P1"
        self._start_time = time.time()

        # v7.5.x: arm the plate-bottom floor for the duration of the run so
        # no Z move (print height, per-object/per-layer offset, …) can punch
        # through the plate. Disarmed in _execute_loop's finally.
        self._arm_print_floor(True)

        # v7.1 P8.4: Auto-start recording
        self._start_recorder()

        # v7.5.x: open the JSONL execution log for this run
        self._begin_exec_log(mode="discrete")

        # v7.1 P8.2: Create trajectory executor with recorder
        self._trajectory_executor = TrajectoryExecutor(
            self.controller, recorder=self.recorder,
            exec_logger=self.exec_logger,
        )

        # v7.1 P8.6: Create service sequence executor
        well_setup = getattr(self.job, 'well_setup', None)
        self._service_executor = ServiceSequenceExecutor(
            self.controller, well_setup=well_setup
        )

        self._thread = threading.Thread(target=self._execute_loop, daemon=True)
        self._thread.start()
        self._set_state(PrintState.RUNNING)

    def resume_from_saved(self, resume_data: dict) -> bool:
        """
        Enhancement 3: Resume a print from saved progress.

        Args:
            resume_data: Dict from load_print_progress() with
                         'job', 'current_step', 'active_pump'.
        Returns:
            True if resume started successfully.
        """
        if self.state == PrintState.RUNNING:
            logger.warning("Cannot resume: print already running")
            return False

        self.job = resume_data["job"]
        start_step = resume_data["current_step"]
        self._active_pump = resume_data.get("active_pump", "P1")
        self._start_time = time.time()

        logger.info(
            f"Resuming print '{self.job.name}' from step "
            f"{start_step}/{self.job.total_steps}"
        )

        self._abort_flag.clear()
        self._pause_event.set()
        self._current_step = start_step

        # v7.5.x: arm the plate-bottom floor (see start()).
        self._arm_print_floor(True)

        # v7.5.x: execution log for the resumed run
        self._begin_exec_log(mode="discrete-resume")

        self._thread = threading.Thread(
            target=self._execute_loop,
            kwargs={"start_from": start_step},
            daemon=True,
        )
        self._thread.start()
        self._set_state(PrintState.RUNNING)
        return True

    def pause(self):
        """Pause the current print. Completes the current command first."""
        if self.state != PrintState.RUNNING:
            return
        self._pause_event.clear()
        self._set_state(PrintState.PAUSED)
        logger.info("Print paused")

        # Enhancement 3: Save progress on pause
        if self.job:
            save_print_progress(
                self.job, self._current_step, self._active_pump
            )

    def resume(self):
        """Resume a paused print."""
        if self.state != PrintState.PAUSED:
            return
        self._pause_event.set()
        self._set_state(PrintState.RUNNING)
        logger.info("Print resumed")

    #: v7.6: how long the abort worker waits for the print thread to unwind to
    #: its ``finally`` (which does the confirmed raise-only retract) before
    #: performing the backstop retract itself.
    _ABORT_UNWIND_S = 20.0

    def abort(self):
        """Abort the current print: kill ALL motion, then retract raise-only.

        Returns to the caller (usually the GUI thread) immediately — the
        motion kill and the retract run on a daemon worker.

        v7.6 rework. Previously this method set flags and then issued a
        fire-and-forget ``move_z_absolute(travel_z)`` **on the calling thread**,
        which had two defects:
          • it is not raise-only, so with the needle already above the travel
            height the "safety" move DESCENDED (only soft-limit clamped);
          • it took the ZP serial lock, so an abort during an in-flight M400
            froze the GUI for 10–180 s.
        Neither survives: the retract of record is now the print thread's
        ``finally`` → ``_retract_to_safe_z`` (``ensure_retracted_to``,
        polarity-safe, confirmed, never descends), and the worker only performs
        a backstop retract if that thread provably fails to unwind.
        """
        if self.state not in (PrintState.RUNNING, PrintState.PAUSED):
            return
        self._abort_flag.set()
        self._pause_event.set()  # Unblock if paused

        # v7.5.x exec log: record the abort request + where we were
        if self.exec_logger:
            self.exec_logger.log("abort_requested",
                                 step=self._current_step)

        # v7.1: Abort trajectory executor if running
        if self._trajectory_executor:
            self._trajectory_executor.abort()

        self._set_state(PrintState.ABORTED)
        logger.info("Print aborted")

        # Enhancement 3: Save progress on abort for potential resume
        if self.job:
            save_print_progress(
                self.job, self._current_step, self._active_pump
            )

        # v7.1 P8.4: Auto-stop recording
        self._stop_recorder("aborted")

        # Enhancement 6: Record abort to history
        self._record_history("aborted")

        # v7.6: kill motion + guarantee the retract, off the calling thread.
        threading.Thread(target=self._abort_worker,
                         name="print-abort-worker", daemon=True).start()

    def _abort_worker(self):
        """v7.6 daemon: stop every axis, then make sure the needle ends up
        retracted however the print thread behaves."""
        try:
            res = self.controller.abort_all_motion("print_abort")
            if self.exec_logger:
                self.exec_logger.log("abort_motion_killed", **{
                    k: v for k, v in (res or {}).items() if k != "reason"})
            logger.warning(
                "Abort: pump volume dispensed is now INDETERMINATE (a move was "
                "cut mid-stroke) — re-check syringe fill before the next run")
        except Exception:
            logger.exception("abort_all_motion failed")

        t = getattr(self, "_thread", None)
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=self._ABORT_UNWIND_S)
            if t.is_alive():
                # The print thread is wedged (e.g. still holding the ZP serial
                # lock). Its finally has provably NOT run, so do the retract
                # here — raise-only, and never with an abort event.
                logger.error(
                    f"Print thread did not unwind {self._ABORT_UNWIND_S:.0f}s "
                    f"after abort — performing the backstop retract")
                self._retract_to_safe_z("abort_backstop")

    # ── Internal ───────────────────────────────────────────────────

    def _set_state(self, new_state: PrintState):
        self.state = new_state
        if self.on_state_changed:
            self.on_state_changed(new_state)

    def _report_progress(self, message: str):
        if self.on_progress:
            self.on_progress(self._current_step, self.job.total_steps, message)

    def _execute_loop(self, start_from: int = 0):
        """Main execution loop running in a dedicated thread."""
        logger.info(f"Starting print: {self.job.name}" +
                     (f" (resuming from step {start_from})" if start_from else ""))

        # v7.5.x: remember whether the ZP board was present at the start so we
        # can ABORT (not silently dry-run) if it drops mid-print. Every ZP move
        # no-ops on zp_stage=None, so without this a disconnect produces a bogus
        # "completed" with no Z/pump motion (observed in the logs).
        self._zp_connected_at_start = getattr(
            self.controller, "is_zp_connected", False)

        # Session 4: Log print start
        pos_logger = getattr(self.controller, 'position_logger', None)
        if pos_logger:
            pos_logger.record(
                "print_start" if start_from == 0 else "print_resume",
                xy_pos=self.controller.get_xy_position(cached=False),
                zp_pos=self.controller.get_zp_position_logical_tuple(cached=False),
                metadata={"job_name": self.job.name, "total_steps": self.job.total_steps,
                          "start_from": start_from},
            )

        try:
            for i, cmd in enumerate(self.job.commands):
                # Skip already-completed commands when resuming
                if i < start_from:
                    continue

                # Check abort
                if self._abort_flag.is_set():
                    logger.info("Abort flag detected, stopping")
                    return

                # Wait if paused
                self._pause_event.wait()

                if self._abort_flag.is_set():
                    return

                # v7.5.x: if the ZP board was present at the start but has
                # dropped mid-print, ABORT rather than dry-run the rest of the
                # plan against a dead board (which no-ops every ZP move and
                # logs a misleading "completed"). Surfaces the disconnect in the
                # execution log so the exact step/time is captured.
                if (self._zp_connected_at_start
                        and not getattr(self.controller,
                                        "is_zp_connected", True)):
                    msg = "ZP stage disconnected during print — aborting"
                    logger.error(msg)
                    if self.exec_logger:
                        self.exec_logger.log(
                            "zp_disconnected", step=i + 1,
                            total=self.job.total_steps)
                    self._set_state(PrintState.ERROR)
                    self._report_progress(msg)
                    self._record_history("error",
                                         error_message="ZP disconnected")
                    self._stop_recorder("error")
                    return

                self._current_step = i + 1
                label = cmd.label or cmd.type.value
                self._report_progress(f"[{i + 1}/{self.job.total_steps}] {label}")

                # v7.5.x exec log: command boundary + duration
                _lg = self.exec_logger
                if _lg:
                    _lg.log("command_start", i=i + 1,
                            total=self.job.total_steps,
                            type=cmd.type.value, label=label)
                _cmd_t0 = time.monotonic()

                self._execute_command(cmd)

                if _lg:
                    _lg.log("command_end", i=i + 1, type=cmd.type.value,
                            duration_s=round(time.monotonic() - _cmd_t0, 3))

                # Session 4: Log position periodically (every 10 commands)
                if pos_logger and i % 10 == 0:
                    pos_logger.record(
                        "print_progress",
                        xy_pos=self.controller.get_xy_position(cached=True),
                        zp_pos=self.controller.get_zp_position_logical_tuple(cached=True),
                        metadata={"step": i + 1, "command": cmd.type.value},
                    )

                # Enhancement 3: Save progress periodically for resume
                if i % self._resume_interval == 0:
                    save_print_progress(
                        self.job, i + 1, self._active_pump
                    )

            self._set_state(PrintState.COMPLETED)
            self._report_progress("Print complete!")
            logger.info("Print job completed successfully")

            # Enhancement 3: Clear resume file on successful completion
            clear_print_progress()

            # Session 4: Log print end
            if pos_logger:
                pos_logger.record(
                    "print_end",
                    xy_pos=self.controller.get_xy_position(cached=False),
                    zp_pos=self.controller.get_zp_position_logical_tuple(cached=False),
                    metadata={"job_name": self.job.name, "result": "completed"},
                )

            # Enhancement 6: Record to print history
            self._record_history("completed")

            # v7.1 P8.4: Auto-stop recording on completion
            self._stop_recorder("completed")

        except Exception as e:
            logger.error(f"Print execution error: {e}", exc_info=True)
            if self.exec_logger:
                self.exec_logger.log_error(str(e), e)
            self._set_state(PrintState.ERROR)
            self._report_progress(f"Error: {e}")

            # Enhancement 3: Save progress on error for potential resume
            save_print_progress(
                self.job, self._current_step, self._active_pump
            )

            if pos_logger:
                pos_logger.record(
                    "print_error",
                    xy_pos=self.controller.get_xy_position(cached=True),
                    zp_pos=self.controller.get_zp_position_logical_tuple(cached=True),
                    metadata={"job_name": self.job.name, "error": str(e)},
                )

            # Enhancement 6: Record error to history
            self._record_history("error", error_message=str(e))

            # v7.1 P8.4: Auto-stop recording on error
            self._stop_recorder("error")

        finally:
            # v7.5.x CRITICAL SAFETY: ALWAYS leave the needle at the safe /
            # travel Z, no matter how the print ended — normal completion, an
            # exception, or the abort early-returns above. Raise-only and
            # idempotent, so for a plan that already ended with TRAVEL_UP this
            # is a confirmed no-op; for an error/abort mid-pattern it lifts the
            # needle out of the well. Done while the exec log is still open so
            # it is recorded.
            self._retract_to_safe_z("loop_end")
            # v7.5.x: disarm the plate-bottom floor (covers completion,
            # error, and the abort early-returns) so it never lingers into
            # subsequent calibration / jogging.
            self._arm_print_floor(False)
            # v7.5.x: always close the execution log (covers the abort
            # early-returns too; status derives from the final state).
            self._end_exec_log()

    def _arm_print_floor(self, active: bool) -> None:
        """v7.5.x: arm/disarm the controller's plate-bottom Z floor."""
        ctrl = self.controller
        if ctrl is not None and hasattr(ctrl, "set_print_floor_active"):
            try:
                ctrl.set_print_floor_active(active)
            except Exception as e:
                logger.debug(f"set_print_floor_active({active}) failed: {e}")

    def _record_history(self, state: str, error_message: str = ""):
        """Enhancement 6: Record completed/aborted/error print to history."""
        if self.print_history is None or self.job is None:
            return
        duration = time.time() - self._start_time if self._start_time else 0.0
        settings_dict = {
            fname: getattr(self.job.settings, fname)
            for fname in self.job.settings.__dataclass_fields__
            if not isinstance(getattr(self.job.settings, fname), dict)
        }
        # Count unique pumps used
        pumps = set()
        for cmd in self.job.commands:
            if cmd.params.get("pump"):
                pumps.add(cmd.params["pump"])
        try:
            self.print_history.add_entry(
                job_name=self.job.name,
                state=state,
                source_file=self.job.source_file,
                total_commands=self.job.total_steps,
                completed_commands=self._current_step,
                duration_seconds=duration,
                settings=settings_dict,
                layers=self.job.settings.num_layers,
                pumps_used=sorted(pumps),
                error_message=error_message,
            )
        except Exception as e:
            logger.warning(f"Failed to record print history: {e}")

    def _retract_for_travel(self, context: str,
                            target_z: float | None = None,
                            feedrate_mm_min: float | None = None) -> None:
        """v7.5.x CRITICAL SAFETY: retract the needle to the travel / "move" Z
        before a cross-position XY move (``MOVE_XY`` / ``HOME_XY``).

        Travel commands move the stage to a DIFFERENT location, so the needle
        must be retracted first and Z must be CONFIRMED there before XY starts —
        independent of any separate ``TRAVEL_UP`` command in the plan
        (defense-in-depth; fixes the Quick-Print "returned to 0,0 without
        retracting" failure). Delegates to
        :meth:`StageController.ensure_retracted_to`, which is polarity-safe
        (ZDIR=±1) and NEVER lowers the needle, so a misconfigured/too-low
        travel-Z degrades to a no-op rather than a crash.

        ``target_z`` (zero-ref mm) overrides the full travel height — used for a
        SMALL intra-well hop between nearby objects (just clears the printed
        material). ``ensure_retracted_to`` still only raises and is floored by
        the insert clearance, so a small target can never cause a descent or a
        crash into a tall insert.

        ``PRINT_PATH`` (within-well) moves are intentionally NOT routed through
        here — Z stays at print height for the print pattern itself.
        """
        ctrl = self.controller
        if not getattr(ctrl, "is_zp_connected", False):
            return
        if not hasattr(ctrl, "ensure_retracted_to"):
            return  # older controller — plan's TRAVEL_UP still applies
        travel_z = (target_z if target_z is not None
                    else getattr(self.job.settings, "travel_z_height", None))
        if travel_z is None:
            return
        if self.exec_logger:
            self.exec_logger.log("z_move",
                                 context=f"retract_for_travel:{context}",
                                 z_mm=round(float(travel_z), 4))
        # v7.5.x: a per-line hop may request a FAST retract feedrate. When none
        # is given, call exactly as before (default retract feedrate) so the
        # common full-travel retract is byte-identical for every caller.
        if feedrate_mm_min is not None:
            try:
                ok = ctrl.ensure_retracted_to(
                    float(travel_z), feedrate_mm_min=feedrate_mm_min)
            except TypeError:
                # Older controller without the feedrate kwarg.
                ok = ctrl.ensure_retracted_to(float(travel_z))
        else:
            ok = ctrl.ensure_retracted_to(float(travel_z))
        if not ok:
            logger.warning("Retract-for-travel (%s): needle not confirmed at "
                           "travel Z — XY move may be unsafe", context)

    def _retract_to_safe_z(self, context: str = "print_end",
                           travel_z: float | None = None) -> None:
        """v7.5.x CRITICAL SAFETY: unconditionally retract the needle to the
        travel / safe Z at the END of a print — on completion, error, OR abort
        — regardless of execution mode (discrete / hybrid / trajectory) and
        regardless of whether the plan happened to end with a retract step.

        This is the single guarantee that *every* print always leaves the
        needle at a safe Z before the operator is free to jog, recalibrate, or
        start the next run (a print that errors or is aborted mid-pattern would
        otherwise leave the needle down in the well at print height). It is
        polarity-safe and RAISE-ONLY (delegates to
        :meth:`StageController.ensure_retracted_to`): it never lowers the
        needle, so when the plan already retracted (the normal completion case)
        it is a confirmed no-op, and a misconfigured/too-low travel-Z degrades
        to a no-op rather than a crash-down.

        Best-effort — never raises (the print is already ending). ``travel_z``
        (zero-ref mm) overrides the height; otherwise the job's
        ``travel_z_height`` is used. ``move_z_absolute`` is the fallback only on
        an older controller without ``ensure_retracted_to``.
        """
        ctrl = self.controller
        try:
            if not getattr(ctrl, "is_zp_connected", False):
                return
            if travel_z is None:
                travel_z = (getattr(self.job.settings, "travel_z_height", None)
                            if self.job else None)
            if travel_z is None:
                return
            if self.exec_logger:
                try:
                    self.exec_logger.log(
                        "z_move", context=f"retract_to_safe_z:{context}",
                        z_mm=round(float(travel_z), 4))
                except Exception:
                    pass
            if hasattr(ctrl, "ensure_retracted_to"):
                # ⚠ v7.6: deliberately NO abort_event here. This is the safety
                # retract of record — it must run to completion even (indeed
                # especially) when the abort flag is set.
                ctrl.ensure_retracted_to(float(travel_z))
            else:  # older controller — best-effort raise (no confirm)
                ctrl.move_z_absolute(float(travel_z), from_zero_ref=True)
        except Exception as e:
            logger.error("Final safe-Z retract (%s) failed: %s", context, e)

    # ── v7.6: abort-aware wrappers for the blocking primitives ─────
    #
    # Each forwards this print's ``_abort_flag`` so an abort unwinds the print
    # thread in ~one readline instead of 10–180 s, and degrades gracefully on a
    # fake / older controller that doesn't accept the keyword. The SAFETY
    # retract (``_retract_to_safe_z``) deliberately does NOT use these — it must
    # always run to completion.

    def _flush_moves(self, zp, timeout_s: float) -> bool:
        try:
            return bool(zp.flush_moves(timeout_s=timeout_s,
                                       abort_event=self._abort_flag))
        except TypeError:
            return bool(zp.flush_moves(timeout_s=timeout_s))

    def _wait_z(self, ctrl, z_mm: float, timeout_s: float) -> bool:
        try:
            return bool(ctrl.wait_for_z_arrival(float(z_mm),
                                                timeout_s=timeout_s,
                                                abort_event=self._abort_flag))
        except TypeError:
            return bool(ctrl.wait_for_z_arrival(float(z_mm),
                                                timeout_s=timeout_s))

    def _pump_uL(self, ctrl, pump, volume_uL, rate_uL_s=None, **kw):
        """Abort-aware ``move_pump_uL`` — used for the DISCRETE actuations
        (prime / retract / suck-back). The streamed print path's tiny
        per-segment emissions stay on the plain call (they are non-blocking and
        already abort-checked per segment/tick)."""
        try:
            return ctrl.move_pump_uL(pump, volume_uL, rate_uL_s,
                                     abort_event=self._abort_flag, **kw)
        except TypeError:
            return ctrl.move_pump_uL(pump, volume_uL, rate_uL_s, **kw)

    def _execute_command(self, cmd: PrintCommand):
        """Execute a single print command."""
        ctrl = self.controller
        settings = self.job.settings
        p = cmd.params

        if cmd.type == CommandType.COMMENT:
            logger.debug(f"Comment: {cmd.label}")
            return

        elif cmd.type == CommandType.MOVE_XY:
            x, y = p.get("x", 0), p.get("y", 0)
            # v7.5.x CRITICAL SAFETY: MOVE_XY is travel to a DIFFERENT location
            # (well start / pen-up move). Guarantee the needle is retracted (and
            # confirmed there) BEFORE the XY move — do not rely solely on a
            # separate preceding TRAVEL_UP. Never descends.
            #
            # A "hop_z" param requests a SMALL intra-well hop (a few mm above
            # print Z) instead of the full travel retract — used between nearby
            # objects in one well so the needle clears the thin printed material
            # without a slow full retract / re-approach.
            hop_z = p.get("hop_z", None)
            if hop_z is not None:
                # v7.5.x (F-4): quick-move relief — before this in-well quick
                # move, suck back so flow stops while the needle travels (then
                # retract + hop). The next object's prime re-primes on arrival.
                self._print_pump_suckback("quick_move")
                # v7.5.x: an inter-line hop may carry a FAST lift feedrate.
                self._retract_for_travel(
                    "move_xy_hop", target_z=float(hop_z),
                    feedrate_mm_min=p.get("retract_feedrate_mm_min"))
            else:
                self._retract_for_travel("move_xy")
            # v7.2.7: Set travel speed before XY move. v7.5.x: an inter-line hop
            # may override with a faster per-line XY speed.
            _tspd = getattr(self.job.settings, 'travel_speed_mm_s', 10.0) if self.job else 10.0
            _tspd = float(p.get("xy_speed_mm_s", _tspd))
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                try:
                    if hasattr(ctrl.xy_stage, "set_speed_mm_s"):

                        ctrl.xy_stage.set_speed_mm_s(_tspd)

                    else:

                        ctrl.xy_stage.set_velocity(max(1, min(100, int(_tspd * 1000 / 50000 * 100))))
                except Exception:
                    pass
            if self.exec_logger:
                self.exec_logger.log(
                    "xy_cmd",
                    context=("hop" if hop_z is not None else "travel"),
                    speed_mm_s=_tspd,
                    **self.exec_logger.xy_cmd_fields(ctrl, x, y))
            ctrl.move_xy_absolute(x, y, from_zero_ref=True)
            self._wait_for_xy_settle(x, y, timeout=10.0)

            # Session 4: Log move
            pos_logger = getattr(ctrl, 'position_logger', None)
            if pos_logger:
                pos_logger.record("move_xy", xy_pos=ctrl.get_xy_position(cached=True),
                                  metadata={"target_x": x, "target_y": y})

        elif cmd.type == CommandType.MOVE_Z:
            z = p.get("z", 0)
            if self.exec_logger:
                self.exec_logger.log("z_move", context="move_z",
                                     z_mm=round(float(z), 4))
            # v7.5.x: drive the descent at the controller's INSERT feedrate (a
            # moderate, deterministic speed) rather than inheriting whatever
            # feedrate the previous command happened to leave set — so the
            # descent's duration is bounded and predictable (shorter on-time).
            # v7.5.x: an inter-line hop's lower-down may carry a FAST line-move
            # Z feedrate (p["feedrate_mm_min"]); else the deterministic INSERT
            # feedrate (never inherits the previous command's modal F).
            #
            # v7.5.x gentle re-entry: on a real controller EMIT a two-segment
            # descent whose FINAL _descend_slow_dist_mm runs slowly (controlled
            # touch-down onto the plate / into a bead), the bulk at the fast
            # feedrate above. Emit-only — the confirm/abort block below stays the
            # single source of truth (no double-wait). On a mock / older
            # controller (no numeric _descend_slow_dist_mm) fall back to the
            # legacy single move so that block still sees exactly one
            # move_z_absolute. The hop's FAST per-line feedrate drives the fast
            # leg; the final mm always runs at _descend_slow_feedrate.
            _z_fr = p.get(
                "feedrate_mm_min", getattr(ctrl, "_zp_insert_feedrate", None))
            if isinstance(getattr(ctrl, "_descend_slow_dist_mm", None),
                          (int, float)) and hasattr(ctrl, "emit_descent_moves"):
                ctrl.emit_descent_moves(z, _z_fr)
            else:
                ctrl.move_z_absolute(z, from_zero_ref=True, feedrate_mm_min=_z_fr)
            # v7.5.x print-setup routine step 3: CONFIRM the needle physically
            # reached the print Z (M400 + position poll) BEFORE the next step
            # (prime / print) — do not start extruding/printing until the Z move
            # has actually completed. Mirrors safe_travel_to's Z verification.
            # The poller is suspended for the wait so its M114 reads don't race
            # the M400/M114 verification. Degrades gracefully to a fixed settle
            # when the controller can't confirm (older controller / mock / no ZP).
            if (getattr(ctrl, "is_zp_connected", False)
                    and hasattr(ctrl, "wait_for_z_arrival")):
                # v7.5.x: SIZE the confirm timeout to the descent's estimated
                # duration. The gentle "slow last mm" re-entry runs its final
                # leg at a low feedrate (e.g. 1 mm @ 6 mm/min = 10 s), so a
                # FIXED 10 s M400 wait expires while a perfectly healthy slow
                # descent is still finishing and FALSELY trips the "board stuck"
                # abort below. Floor at 10 s, add margin, cap as a backstop.
                # (Mirrors move_pump_uL(settle=True)'s duration-scaled drain.)
                _confirm_to = 10.0
                if hasattr(ctrl, "estimate_gentle_z_time_s"):
                    try:
                        _confirm_to = max(
                            10.0,
                            float(ctrl.estimate_gentle_z_time_s(float(z), _z_fr))
                            + 5.0)
                        _confirm_to = min(_confirm_to, 120.0)
                    except Exception:
                        _confirm_to = 10.0
                _had_poller = hasattr(ctrl, "suspend_position_poller")
                if _had_poller:
                    ctrl.suspend_position_poller()
                _z_confirmed = True
                try:
                    zp = getattr(ctrl, "zp_stage", None)
                    _m400_ok = True
                    if zp is not None and hasattr(zp, "flush_moves"):
                        # v7.6: abort-aware — a print aborted during the
                        # print-height confirm used to hold the thread here for
                        # up to 120 s before the retract could run.
                        _m400_ok = self._flush_moves(zp, _confirm_to)
                    if not _m400_ok or not self._wait_z(ctrl, float(z),
                                                        _confirm_to):
                        _z_confirmed = False
                finally:
                    if _had_poller:
                        ctrl.resume_position_poller()
                if not _z_confirmed:
                    # v7.5.x SAFETY: the needle did NOT confirm at print Z
                    # (board stuck "busy", or dropped). Do NOT continue —
                    # extruding / dragging at the wrong Z against a stuck or
                    # half-dropped board is unsafe and ruins the print. Raise
                    # so _execute_loop aborts cleanly (state→ERROR, history +
                    # recorder closed) and its finally retracts to safe Z.
                    if self.exec_logger:
                        self.exec_logger.log(
                            "z_move", context="move_z_abort_unconfirmed",
                            z_mm=round(float(z), 4))
                    raise RuntimeError(
                        f"Z move to print height {float(z):.3f} mm not "
                        "confirmed (board stuck or disconnected) — aborting "
                        "before extrusion")
            else:
                time.sleep(0.5)

        elif cmd.type == CommandType.MOVE_Z_REL:
            dist = p.get("distance", 0)
            feedrate = p.get("feedrate", settings.z_feedrate)
            ctrl.move_z_relative(dist, feedrate)
            time.sleep(0.3)

        elif cmd.type == CommandType.DISPENSE:
            pump = p.get("pump", self._active_pump)

            # v7.2: Check for µL-based command first
            if "amount_uL" in p:
                amount_uL = p["amount_uL"]
                rate_uL_s = p.get("rate_uL_s", settings.get_pump_rate(pump))
                if self.exec_logger:
                    self.exec_logger.log("extrude", context="command",
                                         pump=pump,
                                         vol_uL=round(float(amount_uL), 4),
                                         rate_uL_s=rate_uL_s)
                # v7.5.x: discrete actuation → settle=True. move_pump_uL now
                # brackets the move with the configured pump settle dwell AND
                # blocks for completion (subsuming the manual wait below).
                # compensate=False: the print path owns its own prime (take-up)
                # and _print_pump_suckback (unload), so this discrete
                # prime/dispense must NOT be auto-bracketed by backlash comp.
                _settled = False
                if hasattr(ctrl, 'move_pump_uL'):
                    try:
                        # v7.6: abort-aware (this is the long one — a
                        # multi-needle aspirate can drain for tens of seconds).
                        self._pump_uL(ctrl, pump, amount_uL, rate_uL_s,
                                      settle=True, compensate=False)
                        _settled = True
                    except TypeError:
                        # Older controller / fake without the settle/compensate kwarg.
                        ctrl.move_pump_uL(pump, amount_uL, rate_uL_s)
                else:
                    # Fallback if controller not yet patched
                    logger.warning("Controller missing move_pump_uL — using raw mm")
                    ctrl.move_pump_relative(pump, amount_uL * 0.3, 30.0)
                if not _settled:
                    wait = abs(amount_uL) / max(rate_uL_s, 0.001) + 0.1
                    time.sleep(min(wait, 10.0))

            # v7.1 legacy: mm-based command
            elif "amount" in p:
                amount = p["amount"]
                feedrate = p.get("feedrate", settings.pump_feedrate)
                ctrl.move_pump_relative(pump, amount, feedrate)
                wait = abs(amount) / max(feedrate / 60.0, 0.001) + 0.1
                time.sleep(min(wait, 5.0))

        elif cmd.type == CommandType.PRINT_PATH:
            # v7.5.x ZP-disconnect fix: a PRINT_PATH issues a dense per-segment
            # ZP write stream (one pump G0 per segment, ~1 write/100 ms). With
            # the background poller ACTIVE its M114 reads contend with that
            # stream, fail to parse, and after ~2.5 s of consecutive failures
            # the liveness watchdog FALSE-POSITIVES a "ZP disconnected" — the
            # exact mid-print disconnect seen in the logs. Suspend the poller
            # for the path (its cached position is display-only here; the print
            # is open-loop), and ALWAYS resume in finally so an abort/exception
            # cannot leave polling frozen. resume() also resets the fail window.
            _has_poller = hasattr(ctrl, "suspend_position_poller")
            _has_wd = hasattr(ctrl, "suspend_zp_watchdog")
            if _has_poller:
                ctrl.suspend_position_poller()
            # v7.5.x: also pause the port-health watchdog so NOTHING but the
            # print thread touches the ZP COM handle during the dense burst
            # (its in_waiting/ClearCommError racing a flow-control-paused write
            # is a USB fault surface). Resumed in finally.
            if _has_wd:
                ctrl.suspend_zp_watchdog()
            try:
                self._execute_print_path(cmd)
            finally:
                if _has_poller:
                    ctrl.resume_position_poller()
                if _has_wd:
                    ctrl.resume_zp_watchdog()

        elif cmd.type == CommandType.DWELL:
            seconds = p.get("seconds", 0)
            end_time = time.time() + seconds
            while time.time() < end_time:
                if self._abort_flag.is_set():
                    return
                time.sleep(min(0.1, seconds))

        elif cmd.type == CommandType.TRAVEL_UP:
            if self.exec_logger:
                self.exec_logger.log(
                    "z_move", context="travel_up",
                    z_mm=round(float(settings.travel_z_height), 4))
            # v7.5.x: retract OUT of the print to the travel Z with a GENTLE
            # slow first mm (back-pressure / surface tension can't peel the
            # deposited bead off with the needle at high speed) + confirmed
            # arrival, via ensure_retracted_to (raise-only, polarity-safe,
            # insert-floored, slow-then-fast). This is the inter-well /
            # end-of-print lift. Falls back to an explicit-feedrate move (never
            # a bare G0 Z — the ~12-min-crawl ZP-hang bug) on an older
            # controller without ensure_retracted_to.
            if hasattr(ctrl, "ensure_retracted_to"):
                ctrl.ensure_retracted_to(float(settings.travel_z_height))
            else:
                ctrl.move_z_absolute(
                    settings.travel_z_height, from_zero_ref=True,
                    feedrate_mm_min=getattr(ctrl, "_zp_retract_feedrate", None))
                time.sleep(0.5)

        elif cmd.type == CommandType.TRAVEL_DOWN:
            if self.exec_logger:
                self.exec_logger.log(
                    "z_move", context="travel_down",
                    z_mm=round(float(settings.print_z_height), 4))
            # v7.5.x: explicit INSERT (descent) feedrate — see TRAVEL_UP.
            ctrl.move_z_absolute(
                settings.print_z_height, from_zero_ref=True,
                feedrate_mm_min=getattr(ctrl, "_zp_insert_feedrate", None))
            time.sleep(0.3)

        elif cmd.type == CommandType.HOME_XY:
            # v7.5.x CRITICAL SAFETY: HOME_XY returns to the zero reference — a
            # cross-position travel move. Guarantee the needle is retracted to
            # the travel Z (and confirmed there) BEFORE moving XY home. Fixes
            # the Quick-Print "returned to 0,0 without retracting" failure.
            self._retract_for_travel("home_xy")
            # v7.2.7: Set travel speed before HOME_XY
            _hspd = getattr(self.job.settings, 'travel_speed_mm_s', 10.0) if self.job else 10.0
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                try:
                    if hasattr(ctrl.xy_stage, "set_speed_mm_s"):

                        ctrl.xy_stage.set_speed_mm_s(_hspd)

                    else:

                        ctrl.xy_stage.set_velocity(max(1, min(100, int(_hspd * 1000 / 50000 * 100))))
                except Exception:
                    pass
            if self.exec_logger:
                self.exec_logger.log(
                    "xy_cmd", context="home", speed_mm_s=_hspd,
                    **self.exec_logger.xy_cmd_fields(ctrl, 0.0, 0.0))
            ctrl.move_xy_absolute(0, 0, from_zero_ref=True)
            self._wait_for_xy_settle(0, 0, timeout=15.0)

        elif cmd.type == CommandType.SET_PUMP_RATE:
            pass

        elif cmd.type == CommandType.SWITCH_PUMP:
            self._execute_switch_pump(cmd)

        # v7.1 P8.3: Trajectory execution via MotionController
        elif cmd.type == CommandType.TRAJECTORY:
            self._execute_trajectory(cmd)

        # v7.1 P8.6: Service sequence execution
        elif cmd.type == CommandType.SERVICE_SEQUENCE:
            self._execute_service_sequence(cmd)

        else:
            logger.warning(f"Unknown command type: {cmd.type}")

    def _execute_switch_pump(self, cmd: PrintCommand):
        """
        Session 4: Execute pump switch for multi-material printing.
        
        v7.1 P8.8: Now checks if ink change is needed and triggers
        service sequence automatically.

        Retracts the old pump, switches active pump, then primes the new pump.
        """
        new_pump = cmd.params.get("pump", "P1")
        old_pump = cmd.params.get("old_pump", self._active_pump)
        required_ink = cmd.params.get("ink_name")  # v7.1: optional ink requirement
        settings = self.job.settings
        ctrl = self.controller

        logger.info(f"Switching pump: {old_pump} → {new_pump}")

        # v7.1 P8.8: Check if ink change is needed on the new pump
        if required_ink and self.fluid_tracker.needs_ink_change(new_pump, required_ink):
            logger.info(
                f"Ink change detected for {new_pump}: "
                f"needs '{required_ink}'"
            )
            # Trigger service sequence before switching
            if self._service_executor:
                self._service_executor.execute_sequence(
                    sequence_steps=["waste", "wash", "buffer", "ink"],
                    pump=new_pump,
                    fluid_tracker=self.fluid_tracker,
                    settings=settings,
                )
            self.fluid_tracker.set_ink(new_pump, required_ink)

        # Retract old pump
        retract = settings.get_retract_amount(old_pump)
        if retract > 0:
            ctrl.move_pump_relative(old_pump, -retract, settings.pump_feedrate)
            wait = retract / (settings.pump_feedrate / 60.0) + 0.1
            time.sleep(min(wait, 3.0))

        # Update active pump
        self._active_pump = new_pump

        # Prime new pump
        prime = settings.get_prime_amount(new_pump)
        if prime > 0:
            ctrl.move_pump_relative(new_pump, prime, settings.pump_feedrate)
            wait = prime / (settings.pump_feedrate / 60.0) + 0.1
            time.sleep(min(wait, 3.0))

    # ── v7.1: Trajectory Execution (P8.3) ─────────────────────────

    def _execute_trajectory(self, cmd: PrintCommand):
        """
        P8.3: Execute a trajectory command using the TrajectoryExecutor.

        Params:
            waypoints: list of Waypoint objects (or use job.trajectory_waypoints)
            segment_id: optional segment ID for recording
        """
        p = cmd.params
        waypoints = p.get("waypoints", [])

        # Fallback to job-level trajectory if not specified in command
        if not waypoints and self.job:
            waypoints = self.job.trajectory_waypoints

        if not waypoints:
            logger.warning("TRAJECTORY command but no waypoints provided")
            return

        if self._trajectory_executor is None:
            logger.error("TrajectoryExecutor not initialised")
            return

        self._trajectory_executor.reset()
        success = self._trajectory_executor.execute(
            waypoints=waypoints,
            pause_event=self._pause_event,
            on_progress=lambda cur, total, msg: self._report_progress(
                f"Trajectory: {msg}"
            ),
        )

        if not success:
            logger.info("Trajectory execution was aborted")

    # ── v7.1: Service Sequence Execution (P8.6) ──────────────────

    def _execute_service_sequence(self, cmd: PrintCommand):
        """
        P8.6: Execute a service sequence command.

        Params:
            pump: Which pump to service
            steps: List of step names (e.g. ["waste", "wash", "buffer", "ink"])
        """
        p = cmd.params
        pump = p.get("pump", self._active_pump)
        steps = p.get("steps", ["waste", "wash", "buffer", "ink"])

        if self._service_executor is None:
            logger.warning("ServiceSequenceExecutor not available")
            return

        self._service_executor.execute_sequence(
            sequence_steps=steps,
            pump=pump,
            fluid_tracker=self.fluid_tracker,
            settings=self.job.settings if self.job else None,
            on_progress=lambda step, idx, total: self._report_progress(
                f"Service: {step} ({idx+1}/{total})"
            ),
        )

    # ── v7.1: Recording Helpers (P8.4) ───────────────────────────

    # ── v7.5.x: Execution log helpers ─────────────────────────────

    def _begin_exec_log(self, mode: str = "discrete") -> None:
        """Open a fresh JSONL execution log for this run (never raises).

        If an exec_logger was injected externally and is already active
        (e.g. app.py opened one for hybrid/trajectory mode), reuse it.
        """
        try:
            if self.exec_logger is not None and self.exec_logger.active:
                return
            self.exec_logger = PrintExecutionLogger(
                job_name=self.job.name if self.job else "print", mode=mode)
            self.exec_logger.start(
                self.controller,
                PrintExecutionLogger.manifest_for_job(
                    self.job, self.controller, mode))
        except Exception as e:
            logger.warning(f"Execution log unavailable: {e}")
            self.exec_logger = None

    def _end_exec_log(self) -> None:
        """Close the execution log with a status from the final state."""
        if self.exec_logger is None:
            return
        try:
            status = {
                PrintState.COMPLETED: "completed",
                PrintState.ABORTED: "aborted",
                PrintState.ERROR: "error",
            }.get(self.state, self.state.name.lower())
            self.exec_logger.stop(status)
        except Exception:
            pass

    def _start_recorder(self) -> None:
        """P8.4: Auto-start recording when print begins."""
        if self.recorder is None:
            return
        try:
            workspace_dict = None
            well_setup_dict = None
            if self.job:
                ws = getattr(self.job, 'workspace', None)
                if ws and hasattr(ws, 'to_dict'):
                    workspace_dict = ws.to_dict()
                wsu = getattr(self.job, 'well_setup', None)
                if wsu and hasattr(wsu, 'to_dict'):
                    well_setup_dict = wsu.to_dict()

            self.recorder.start_recording(
                workspace_dict=workspace_dict,
                well_setup_dict=well_setup_dict,
                job_name=self.job.name if self.job else "unnamed",
            )
            logger.info("PrintRecorder: auto-started for print")
        except Exception as e:
            logger.warning(f"Failed to start recording: {e}")

    def _stop_recorder(self, status: str) -> None:
        """P8.4: Auto-stop recording when print ends."""
        if self.recorder is None:
            return
        try:
            if hasattr(self.recorder, 'is_recording') and self.recorder.is_recording:
                self.recorder.stop_recording(status=status)
                logger.info(f"PrintRecorder: auto-stopped (status={status})")
        except Exception as e:
            logger.warning(f"Failed to stop recording: {e}")

    def _set_xy_speed_for_print(self, speed_mm_s: float = 0):
        """v7.2.7: use set_speed_mm_s — Set Prior XY stage speed before print."""
        settings = self.job.settings if self.job else None
        if speed_mm_s <= 0 and settings:
            speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)
        if speed_mm_s <= 0 and settings:
            speed_mm_s = max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        if speed_mm_s <= 0:
            speed_mm_s = 5.0
        ctrl = self.controller
        if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
            # v7.5.x: set ACCELERATION too — short print segments are
            # accel-dominated, and the Prior otherwise uses whatever (often
            # low) accel was last set, so moves take far longer than
            # seg_len/speed implies. A brisk accel keeps short moves crisp.
            if hasattr(ctrl.xy_stage, 'set_acceleration'):
                try:
                    ctrl.xy_stage.set_acceleration(
                        getattr(self.job.settings, 'xy_accel_pct', 80)
                        if self.job else 80)
                except Exception:
                    pass
            if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                ctrl.xy_stage.set_speed_mm_s(speed_mm_s)
            else:
                ctrl.xy_stage.set_velocity(speed_mm_s * 1000)   # µm/s → SMS%
            logger.info(f"Print speed set: {speed_mm_s:.1f} mm/s")
            if self.exec_logger:
                self.exec_logger.log("speed_set", context="print_path",
                                     mm_s=round(speed_mm_s, 3))


    def _print_pump_suckback(self, context: str, pump: str | None = None) -> None:
        """v7.5.x: suck back the per-pump compliance / relief volume during ink
        printing to stop drool — after a deposit (``context='deposit'``, end of a
        print path) and before a quick / inter-object move
        (``context='quick_move'``). This is the print-path "unload" half of
        backlash compensation, gated on the single ``backlash_comp_enabled``
        toggle; the volume is the per-pump ``pump_relief_uL`` (µL measured by the
        Needle Location compliance calibration). Best-effort and exception-safe —
        never breaks a print. Re-priming after a quick move is handled by the next
        object's existing prime dispense (the print-start "take-up")."""
        ctrl = self.controller
        pump = pump or getattr(self, "_active_pump", None)
        if not pump:
            return
        relief_fn = getattr(ctrl, "pump_relief_uL", None)
        enabled_fn = getattr(ctrl, "backlash_comp_enabled", None)
        if not (callable(relief_fn) and callable(enabled_fn)):
            return  # older controller / fake without the compliance readers
        try:
            if not enabled_fn():
                return
            r = float(relief_fn(pump) or 0.0)
            if r <= 0:
                return
            # This IS the explicit unload; move_pump_uL(settle=False) never
            # auto-brackets, so it won't recursively re-suck-back.
            self._pump_uL(ctrl, pump, -r)   # ASPIRATE = suck-back (v7.6 abort-aware)
            if self.exec_logger:
                self.exec_logger.log("pump_relief", context=context,
                                     uL=round(-r, 4), pump=pump)
            logger.debug(f"print suck-back ({context}): {-r:+.4f} µL on {pump}")
        except Exception as e:   # pragma: no cover - defensive
            logger.debug(f"print suck-back ({context}) skipped: {e}")

    def _execute_print_path(self, cmd: PrintCommand):
        """
        Execute a coordinated print path: move XY while extruding.
        """
        ctrl = self.controller
        settings = self.job.settings
        points = cmd.params.get("points", [])
        pump = cmd.params.get("pump", self._active_pump)
        # v7.2: flow_rate_uL_s takes priority, fall back to legacy flow_rate
        flow_rate_uL_s = cmd.params.get("flow_rate_uL_s", None)
        flow_rate = cmd.params.get("flow_rate", 0.01)
        use_uL = flow_rate_uL_s is not None
        # v7.5.x: confirmed per-segment printing — wait for the stage to
        # physically arrive (and drain the pump board) after each segment so the
        # open-loop stream can never outrun the stage. See PrintSettings.
        confirm_each_segment = bool(getattr(settings, "confirm_each_segment", False))

        if len(points) < 2:
            return

        # v7.5.x: confirmed-per-segment now stops (wait-for-arrival + pump drain)
        # ONLY at real CORNERS, not at every sampled node. The incoming `points`
        # is a fine polyline; waiting at each collinear node made straight edges
        # crawl (stop-and-go). Precompute which vertices are corners (turn angle
        # over the threshold) so a star stops at its 10 tips and streams the
        # straight edges between them. The last point is always a stop.
        _confirm_corners = None
        if confirm_each_segment:
            _cc_angle = getattr(settings, "confirm_corner_angle_deg", 0) or 30.0
            try:
                _confirm_corners = _velctl.corner_flags(points, _cc_angle)
            except Exception:
                _confirm_corners = None

        # v7.5.x: CLOSED-LOOP velocity following (preferred). Continuously polls
        # the real position and re-commands a velocity vector toward a carrot
        # ahead of the stage's ACTUAL progress — so a slow stage can't lag and
        # the pump tracks real distance. Requires a continuous-velocity command
        # (Prior VS via send_velocity_xy); otherwise fall through to the discrete
        # path below.
        if (bool(getattr(settings, "velocity_follow", False))
                and hasattr(ctrl, "send_velocity_xy")
                and getattr(ctrl, "is_xy_connected", False)):
            self._execute_print_path_velocity(cmd)
            return

        # v7.5.x: OPEN-LOOP velocity streaming ("open_loop" mode). Feed a
        # continuous velocity vector along the path tangent (no position reads,
        # no point-to-point moves — that's confirm_each_segment). Requires a
        # continuous-velocity command; otherwise fall through to the discrete
        # point-stream path below.
        if (bool(getattr(settings, "velocity_open_loop", False))
                and hasattr(ctrl, "send_velocity_xy")
                and getattr(ctrl, "is_xy_connected", False)):
            self._execute_print_path_open_velocity(cmd)
            return

        # v7.2.7: Set stage speed before print path
        self._set_xy_speed_for_print()

        # v7.5.x exec log: path manifest + drift bookkeeping. planned_s
        # accumulates the loop's own schedule (sleep budget); wall drift
        # beyond it = serial/computation overhead. The sampler separately
        # captures the *physical* lag of the stage behind the commands.
        lg = self.exec_logger
        _spd = getattr(settings, 'print_speed_mm_s', 0) or \
            max(getattr(settings, 'print_feedrate', 200), 1) / 60.0

        if lg:
            lg.log("path_start", n_points=len(points), pump=pump,
                   flow_rate_uL_s=flow_rate_uL_s, flow_rate=flow_rate,
                   speed_mm_s=round(_spd, 3),
                   confirm_each_segment=confirm_each_segment,
                   **PrintExecutionLogger._path_stats(points),
                   **PrintExecutionLogger.path_points(points))
        _path_t0 = time.monotonic()
        _planned_s = 0.0
        # v7.5.x (Finding C): residual pump volume that hasn't yet crossed the
        # per-emission floor; accumulated across segments, flushed at path end.
        _pending_pump_uL = 0.0

        # Move to start of path
        start_x, start_y = points[0][0], points[0][1]
        if lg:
            lg.log("xy_cmd", context="path_start",
                   **lg.xy_cmd_fields(ctrl, start_x, start_y))
        ctrl.move_xy_absolute(start_x, start_y, from_zero_ref=True)
        self._wait_for_xy_settle(start_x, start_y, timeout=5.0)

        # Execute path segments
        for i in range(1, len(points)):
            if self._abort_flag.is_set():
                if lg:
                    lg.log("path_end", aborted_at_segment=i,
                           wall_s=round(time.monotonic() - _path_t0, 3),
                           planned_s=round(_planned_s, 3))
                return

            # v7.5.x: if ZP dropped mid-path, stop NOW — do not drag the needle
            # dry through the print at print Z (XY is a separate board and would
            # keep moving with the pump dead). The outer _execute_loop logs the
            # disconnect + aborts the job at the next command boundary. Gated on
            # "was connected at start" so a job intentionally run without ZP
            # (e.g. an XY-only sim / log test) is unaffected.
            if (getattr(self, "_zp_connected_at_start", False)
                    and not getattr(ctrl, "is_zp_connected", True)):
                logger.error("PRINT_PATH: ZP disconnected mid-path — stopping")
                if lg:
                    lg.log("path_end", zp_disconnect_at_segment=i,
                           wall_s=round(time.monotonic() - _path_t0, 3),
                           planned_s=round(_planned_s, 3))
                return

            x1, y1 = points[i - 1][0], points[i - 1][1]
            x2, y2 = points[i][0], points[i][1]
            seg_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if seg_length < 0.001:
                continue

            # v7.2: Extrude using µL/s flow rate or legacy ratio
            _seg_vol_uL = 0.0   # v7.5.x exec log: extrusion bookkeeping
            _seg_vol_dropped = False
            if use_uL and flow_rate_uL_s and flow_rate_uL_s > 0:
                # Calculate volume from flow rate × segment time
                # v7.2.7: Use mm/s for extrusion timing
                _ext_speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)
                if _ext_speed_mm_s <= 0:
                    _ext_speed_mm_s = max(settings.print_feedrate, 1.0) / 60.0
                seg_time = seg_length / max(_ext_speed_mm_s, 0.01) if _ext_speed_mm_s > 0 else 0
                volume_uL = flow_rate_uL_s * seg_time
                _seg_vol_uL = volume_uL
                # v7.5.x (Finding C): ACCUMULATE sub-threshold volume instead of
                # dropping it. A single segment's dispense can round below the
                # emit floor (fine paths / thin beads), but the volume is REAL —
                # carry the residual forward and emit one pump move once the
                # pending total is worth moving, so the print is not cumulatively
                # under-extruded (the leftover is flushed at path end). Pacing +
                # logging still use the nominal per-segment volume, so timing is
                # unchanged.
                _pending_pump_uL += volume_uL
                if _pending_pump_uL > self._PATH_PUMP_EMIT_MIN_UL:
                    _emit_uL = _pending_pump_uL
                    _pending_pump_uL = 0.0
                    if hasattr(ctrl, 'move_pump_uL'):
                        ctrl.move_pump_uL(pump, _emit_uL, flow_rate_uL_s)
                    else:
                        # v7.4.2: honor configurable per-machine axis_map
                        _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
                            if ctrl.zp_stage else AXIS_MAP
                        mapped = _axis_map.get(pump, AXIS_MAP.get(pump))
                        if mapped and ctrl.zp_stage:
                            ctrl.zp_stage.move_relative(
                                {mapped: _emit_uL * 0.3},
                                settings.pump_feedrate,
                            )
                else:
                    _seg_vol_dropped = True   # deferred (accumulating residual)
            else:
                # Legacy: extrude proportional to segment length (dimensionless ratio)
                extrude_amount = seg_length * flow_rate
                if extrude_amount > 0.0001:
                    # v7.4.2: honor configurable per-machine axis_map
                    _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
                        if ctrl.zp_stage else AXIS_MAP
                    mapped = _axis_map.get(pump, AXIS_MAP.get(pump))
                    if mapped and ctrl.zp_stage:
                        ctrl.zp_stage.move_relative(
                            {mapped: extrude_amount},
                            settings.pump_feedrate,
                        )

            # Move XY
            _xy_fields = lg.xy_cmd_fields(ctrl, x2, y2) if lg else {}
            ctrl.move_xy_absolute(x2, y2, from_zero_ref=True)
            # v7.2.7: speed_mm_s — use explicit mm/s, fallback to legacy mm/min
            _speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)
            if _speed_mm_s <= 0:
                _speed_mm_s = max(settings.print_feedrate, 1) / 60.0
            move_time = seg_length / max(_speed_mm_s, 0.01)
            # v7.5.x: pace by the RATE-LIMITING axis, not just XY transit. The
            # pump dispense for this segment physically takes
            # _seg_vol_uL / flow_rate_uL_s seconds; if that exceeds the XY move
            # time, the old XY-only sleep UNDER-counted it and the loop admitted
            # pump G0s faster than Marlin could execute them → planner buffer
            # fills → board stalls under flow control → USB write faults.
            pump_move_s = 0.0
            if use_uL and flow_rate_uL_s and flow_rate_uL_s > 0 and _seg_vol_uL > 0:
                pump_move_s = _seg_vol_uL / flow_rate_uL_s
            # v7.5.x: pace the XY component by the tuned correction (>1 for a
            # stage that runs slower than commanded, so the loop doesn't outrun
            # the stage → pump stays locked to the needle). Default 1.0 = legacy.
            _pace = getattr(settings, "pace_correction", 1.0) or 1.0
            _sleep_s = max(move_time * _pace, pump_move_s, 0.05)
            # v7.5.x: in confirm mode, a node is a STOP only if it's a real corner
            # (or the last point). Straight-edge nodes stream (a short sleep like
            # the open path) so a star prints one continuous move per edge and
            # stops only at its corners — not a crawl at every sampled node.
            _is_stop_node = confirm_each_segment and (
                i == len(points) - 1
                or _confirm_corners is None      # detection failed → stop each
                or (i < len(_confirm_corners) and _confirm_corners[i]))
            if _is_stop_node:
                # CLOSED-LOOP pacing at the corner: block until the stage
                # physically reaches this vertex, then drain the pump board,
                # before issuing the next edge. The stream therefore cannot
                # accumulate lag along the edge just traced. Timeout is generous
                # (the stage may be several × slower than move_time/pump_move_s
                # imply); a persistent timeout is logged by _wait_for_xy_settle.
                _seg_settle_to = max(_sleep_s * 4.0 + 2.0, 3.0)
                _seg_tol = (getattr(settings, "segment_settle_tol_um", 0) or 0) \
                    or self._SEGMENT_SETTLE_TOL_UM
                self._wait_for_xy_settle(
                    x2, y2, timeout=_seg_settle_to, tolerance=_seg_tol)
                # Drain the pump board too so its dispense completes in step with
                # the stage and its planner buffer can never back up.
                if getattr(ctrl, "is_zp_connected", False):
                    _zp = getattr(ctrl, "zp_stage", None)
                    if _zp is not None and hasattr(_zp, "flush_moves"):
                        self._flush_moves(_zp, 10.0)
            else:
                time.sleep(_sleep_s)

            # v7.5.x exec log: one line per segment. drift_s = how far the
            # loop's wall clock has run ahead of its own sleep schedule
            # (serial/computation overhead); the physical stage lag shows
            # up separately in the sampler's 'sample' events as lag_um.
            if lg:
                _planned_s += _sleep_s
                _ev = {"i": i, "n": len(points),
                       "seg_mm": round(seg_length, 4),
                       "mv_s": round(move_time, 4),
                       "slp_s": round(_sleep_s, 4),
                       "drift_s": round(
                           (time.monotonic() - _path_t0) - _planned_s, 3)}
                if _seg_vol_uL:
                    _ev["vol_uL"] = round(_seg_vol_uL, 5)
                if _seg_vol_dropped:
                    _ev["vol_dropped"] = True
                _ev.update(_xy_fields)
                lg.log("path_segment", **_ev)

            # v7.5.x: periodic planner-buffer barrier. Marlin's 'ok' = admitted
            # to the planner buffer, NOT move-complete; over a long, finely-
            # sampled path the loop can admit pump G0s faster than the board
            # executes them, fill the ~16-block buffer, stall under flow
            # control, and fault the USB write (the observed "crawl" → ZP drop).
            # Draining the buffer every _PATH_BARRIER_EVERY segments bounds its
            # depth so it can NEVER saturate. The poller + watchdog are already
            # suspended for PRINT_PATH, so this M400 round trip is uncontended;
            # it's near-instant when the buffer is shallow (or ZP has no queued
            # moves, e.g. a dry/low-flow print) and only waits when motion is
            # genuinely backed up — exactly when we need it to.
            # Runs on STREAMED nodes only (open-loop, and confirm's straight-edge
            # nodes that didn't just drain at a stop-node) so a long streamed
            # edge in either mode can't saturate the buffer.
            if (not _is_stop_node
                    and i % self._PATH_BARRIER_EVERY == 0
                    and getattr(ctrl, "is_zp_connected", False)):
                _zp = getattr(ctrl, "zp_stage", None)
                if _zp is not None and hasattr(_zp, "flush_moves"):
                    _bok = self._flush_moves(_zp, 10.0)
                    if lg and not _bok:
                        lg.log("path_barrier", i=i, ok=False)

        # v7.5.x (Finding C): flush any residual accumulated pump volume that
        # never crossed the per-emission floor mid-path, so the tail of the
        # print isn't under-extruded. (Abort / ZP-disconnect take an early
        # return above and correctly skip this.)
        if (use_uL and flow_rate_uL_s and flow_rate_uL_s > 0
                and _pending_pump_uL > 0):
            if hasattr(ctrl, 'move_pump_uL'):
                ctrl.move_pump_uL(pump, _pending_pump_uL, flow_rate_uL_s)
            else:
                _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
                    if ctrl.zp_stage else AXIS_MAP
                mapped = _axis_map.get(pump, AXIS_MAP.get(pump))
                if mapped and ctrl.zp_stage:
                    ctrl.zp_stage.move_relative(
                        {mapped: _pending_pump_uL * 0.3}, settings.pump_feedrate)
            _pending_pump_uL = 0.0

        # v7.5.x SYNC FIX (XY/ZP de-sync at the END of a print): the per-segment
        # XY moves above are OPEN-LOOP streamed — each Prior "G x,y" returns its
        # "R" (received) ack immediately and moves asynchronously, paced only by
        # this loop's sleep, NOT by arrival. So the Prior builds a backlog of
        # queued moves and is still draining them when the segment loop ends.
        # Without waiting here, _execute_command returns, the plan advances to
        # the Z retract / next step, and the stage keeps moving AFTER the print
        # is "done" — the needle retracts out of sync while XY is still tracing
        # the tail of the path (the reported symptom). DRAIN the queue: block
        # until the stage physically reaches the final path point before
        # returning. (cached=False direct query works with the poller suspended;
        # _wait_for_xy_settle honors the abort flag and logs a settle_wait.)
        if points:
            final_x, final_y = points[-1][0], points[-1][1]
            self._wait_for_xy_settle(final_x, final_y, timeout=30.0)

        # v7.5.x (F-4): deposit relief — suck back a little after the deposit so
        # ink doesn't string/drool as the needle lifts or travels. Gated on the
        # deposit toggle; fires once per print path (for the common single-object
        # print that's at the end). Re-primed by the next object's prime.
        self._print_pump_suckback("deposit", pump)

        # v7.5.x exec log: path summary. wall_s now includes the end-of-path XY
        # drain above, so it reflects the TRUE physical path-completion time —
        # the stage is settled at the final point when this fires, so the
        # subsequent Z retract / next command starts in sync with XY.
        if lg:
            lg.log("path_end",
                   wall_s=round(time.monotonic() - _path_t0, 3),
                   planned_s=round(_planned_s, 3))

    def _execute_print_path_open_velocity(self, cmd: PrintCommand):
        """OPEN-LOOP velocity-streaming print path (see PrintSettings.
        velocity_open_loop).

        Feed-forward: a target advances ALONG the path by wall-clock time at the
        commanded speed, and each tick commands a continuous velocity vector
        (Prior ``VS``) pointed along the path TANGENT at the target — "just
        updating the velocity vector needed" to trace the path. There is NO
        position feedback (that is the closed-loop ``velocity_follow`` mode) and
        NO point-to-point stepping (that is ``confirm_each_segment``). The stage
        therefore moves continuously (even ink laydown) but drift is not
        corrected. The pump deposits volume ∝ the time-based target advance.

        Because there is no feedback, there is no runaway guard — the direction
        must be correct (same ``VS`` frame as every other mode). Bounded by a
        wall-time cap and ALWAYS stops the stage (``VS 0,0``) on exit; an
        end-of-path settle drains XY so the retract/next step stays in sync.
        """
        ctrl = self.controller
        settings = self.job.settings
        raw_pts = cmd.params.get("points", [])
        pump = cmd.params.get("pump", self._active_pump)
        flow_rate_uL_s = cmd.params.get("flow_rate_uL_s", None)
        flow_rate = cmd.params.get("flow_rate", 0.01)
        use_uL = flow_rate_uL_s is not None
        lg = self.exec_logger

        pts = [(float(raw_pts[0][0]), float(raw_pts[0][1]))]
        for p in raw_pts[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
                pts.append((float(p[0]), float(p[1])))
        if len(pts) < 2:
            return
        cum = polyline_arclength(pts)
        total = cum[-1]

        print_speed = getattr(settings, 'print_speed_mm_s', 0) or \
            max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        print_speed = max(0.05, float(print_speed))
        # pace_correction (>1) trims the MOTION speed down to a stage that can't
        # hold the full commanded velocity; it stretches the wall time but NOT
        # the deposited volume — vol_per_mm uses the COMMANDED speed (matching
        # the closed-loop follower + the legacy open-loop path), so the bead is
        # the same width at any pace, just laid down slower.
        _pace = getattr(settings, "pace_correction", 1.0) or 1.0
        eff_speed = max(0.05, print_speed / max(1.0, _pace))
        vol_per_mm = (flow_rate_uL_s / print_speed) if (
            use_uL and flow_rate_uL_s and flow_rate_uL_s > 0) else 0.0

        max_um_s = 50000.0
        sl = getattr(ctrl, 'safety_limits', None)
        try:
            _m = float(getattr(sl, 'max_xy_speed', 0) or 0)
            if _m > 0:
                max_um_s = _m
        except Exception:
            pass
        xy = getattr(ctrl, 'xy_stage', None)
        try:
            if xy is not None and hasattr(xy, 'set_acceleration'):
                xy.set_acceleration(getattr(settings, 'xy_accel_pct', 80) or 80)
            if xy is not None and hasattr(xy, 'set_speed_mm_s'):
                xy.set_speed_mm_s(max_um_s / 1000.0)   # SMS 100% so VS can reach
        except Exception:
            pass

        # Move to the path start (confirmed) before streaming velocity.
        if lg:
            lg.log("xy_cmd", context="openvel_path_start",
                   **lg.xy_cmd_fields(ctrl, pts[0][0], pts[0][1]))
        ctrl.move_xy_absolute(pts[0][0], pts[0][1], from_zero_ref=True)
        self._wait_for_xy_settle(pts[0][0], pts[0][1], timeout=5.0)

        if lg:
            lg.log("path_start", n_points=len(pts), pump=pump,
                   flow_rate_uL_s=flow_rate_uL_s, flow_rate=flow_rate,
                   speed_mm_s=round(eff_speed, 3), mode="open_velocity",
                   **PrintExecutionLogger._path_stats(pts),
                   **PrintExecutionLogger.path_points(pts))

        _ctrl_hz = getattr(settings, "vel_control_hz", 0) or self._VEL_CONTROL_HZ
        _decel = getattr(settings, "vel_decel_mm", 0) or self._VEL_DECEL_MM
        dt = 1.0 / max(_ctrl_hz, 1.0)
        _t0 = time.monotonic()
        s_prev = 0.0
        pending_uL = 0.0
        n_ticks = 0
        max_wall = total / eff_speed * 4.0 + 15.0

        try:
            while True:
                _tick = time.monotonic()
                if self._abort_flag.is_set():
                    if lg:
                        lg.log("path_end", mode="open_velocity", aborted=True,
                               s_mm=round(s_prev, 3),
                               wall_s=round(_tick - _t0, 3))
                    return
                if (getattr(self, "_zp_connected_at_start", False)
                        and not getattr(ctrl, "is_zp_connected", True)):
                    logger.error("OPENVEL_PATH: ZP disconnected mid-path — stopping")
                    if lg:
                        lg.log("path_end", mode="open_velocity",
                               zp_disconnect=True, s_mm=round(s_prev, 3),
                               wall_s=round(_tick - _t0, 3))
                    return
                if _tick - _t0 > max_wall:
                    logger.warning("OPENVEL_PATH: wall-time cap at s=%.2f/%.2f mm",
                                   s_prev, total)
                    break

                # Target advances by wall-clock time at the commanded speed.
                elapsed = _tick - _t0
                s_tgt = min(eff_speed * elapsed, total)
                ds = s_tgt - s_prev

                # Deposit pump ∝ the target advance (feed-forward).
                if ds > 0:
                    if vol_per_mm > 0:
                        pending_uL += ds * vol_per_mm
                        if pending_uL > self._PATH_PUMP_EMIT_MIN_UL:
                            self._emit_pump(ctrl, pump, pending_uL,
                                            flow_rate_uL_s, settings)
                            pending_uL = 0.0
                    elif not use_uL and flow_rate:
                        self._emit_pump(ctrl, pump, ds * flow_rate, None, settings)

                # Velocity vector along the path tangent at the target, ramping
                # down over the last _decel for a cleaner (open-loop) stop.
                tx, ty = tangent_at_arclength(pts, cum, s_tgt)
                speed = eff_speed
                remaining = total - s_tgt
                if remaining < _decel:
                    speed *= max(0.1, remaining / _decel)
                vx = speed * tx * 1000.0    # mm/s → µm/s
                vy = speed * ty * 1000.0
                vmag = math.hypot(vx, vy)
                if vmag > max_um_s and vmag > 0:
                    vx *= max_um_s / vmag
                    vy *= max_um_s / vmag
                ctrl.send_velocity_xy(vx, vy)

                n_ticks += 1
                if lg and (n_ticks % 5 == 0):
                    lg.log("openvel_sample", s_mm=round(s_tgt, 3),
                           tot_mm=round(total, 3),
                           vx=round(vx, 0), vy=round(vy, 0))

                s_prev = s_tgt
                if s_tgt >= total:
                    break
                _elapsed = time.monotonic() - _tick
                if _elapsed < dt:
                    time.sleep(dt - _elapsed)
        finally:
            try:
                ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass

        if pending_uL > 0 and vol_per_mm > 0:
            self._emit_pump(ctrl, pump, pending_uL, flow_rate_uL_s, settings)

        # Feed-forward can't guarantee the EXACT end position (no feedback +
        # the stage coasts after VS 0,0), so land precisely on the endpoint with
        # a final positioning move and drain XY, so the Z retract / next command
        # starts in sync. This is termination, not the control method.
        ctrl.move_xy_absolute(pts[-1][0], pts[-1][1], from_zero_ref=True)
        self._wait_for_xy_settle(pts[-1][0], pts[-1][1], timeout=10.0)
        self._print_pump_suckback("deposit", pump)
        if lg:
            lg.log("path_end", mode="open_velocity",
                   wall_s=round(time.monotonic() - _t0, 3),
                   s_mm=round(s_prev, 3))

    def _execute_print_path_velocity(self, cmd: PrintCommand):
        """CLOSED-LOOP velocity-following print path (see PrintSettings.
        velocity_follow).

        A real-time control loop (~_VEL_CONTROL_HZ) that, every tick:
          1. polls the stage's ACTUAL position (direct, non-cached),
          2. projects it onto the toolpath → real arc-length progress ``s``,
          3. deposits pump volume in proportion to the REAL ``Δs`` travelled
             (so the bead is correct no matter how fast/slow the stage moved),
          4. places a "carrot" a fixed lookahead ahead of ``s`` and commands a
             velocity vector (Prior ``VS``) toward it at the print speed,
             ramping down over the last ``_VEL_DECEL_MM`` for a clean stop.

        Because the carrot is tied to the stage's real progress (arc-length
        parametrized, pure pursuit) it can NEVER run ahead of a slow stage — the
        open-loop time-paced path's failure mode — and position feedback pulls
        the stage back onto the path each tick. Always stops the stage (``VS
        0,0``) on every exit. A large sustained cross-track error (e.g. a VS sign
        inversion sending the stage the wrong way) trips a runaway guard →
        RuntimeError → the outer loop aborts + retracts to safe Z.
        """
        # v7.6: the feature-aware feed plan supersedes the single-tuning loop
        # when enabled AND the machine is characterised. It returns False —
        # having issued NO motion — when it cannot run, so we fall through to
        # the legacy loop below (which is byte-identical when the flag is off).
        if getattr(self.job.settings, "feed_plan_enabled", False):
            try:
                if self._execute_print_path_feed_plan(cmd):
                    return
            except RuntimeError:
                raise                       # runaway guard → outer loop aborts
            except Exception as e:
                logger.exception("feed-plan path failed — falling back: %s", e)

        ctrl = self.controller
        settings = self.job.settings
        raw_pts = cmd.params.get("points", [])
        pump = cmd.params.get("pump", self._active_pump)
        flow_rate_uL_s = cmd.params.get("flow_rate_uL_s", None)
        flow_rate = cmd.params.get("flow_rate", 0.01)
        use_uL = flow_rate_uL_s is not None
        lg = self.exec_logger

        # De-dup consecutive coincident points (zero-length segments break the
        # projection's segment math and add nothing).
        pts = [(float(raw_pts[0][0]), float(raw_pts[0][1]))]
        for p in raw_pts[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
                pts.append((float(p[0]), float(p[1])))
        if len(pts) < 2:
            return
        cum = polyline_arclength(pts)
        total = cum[-1]

        print_speed = getattr(settings, 'print_speed_mm_s', 0) or \
            max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        print_speed = max(0.05, float(print_speed))
        vol_per_mm = (flow_rate_uL_s / print_speed) if (
            use_uL and flow_rate_uL_s and flow_rate_uL_s > 0) else 0.0

        # v7.5.x: pursuit + corner + PID tuning — tuned by the XY Printing
        # Challenge and stamped on the settings by Quick Print; 0/absent → class
        # defaults (kp/kd 0 = pure pursuit, legacy behaviour).
        _lookahead = getattr(settings, "vel_lookahead_mm", 0) or self._VEL_LOOKAHEAD_MM
        _decel = getattr(settings, "vel_decel_mm", 0) or self._VEL_DECEL_MM
        _kp = float(getattr(settings, "vel_pid_kp", 0.0) or 0.0)
        _kd = float(getattr(settings, "vel_pid_kd", 0.0) or 0.0)
        _corner_ang = getattr(settings, "vel_corner_angle_deg", 0) \
            or self._VEL_CORNER_ANGLE_DEG
        _corner_fac = getattr(settings, "vel_corner_speed_factor", 0) \
            or self._VEL_CORNER_SPEED_FACTOR

        # v7.5.x: ground the loop rate + VS ceiling + dead-time SPEED CAP in the
        # MACHINE-MEASURED calibration (control-loop period, true max speed, phase
        # lag) stamped by Quick Print. 0/absent → safety-envelope / class-constant
        # fallbacks (legacy). The dead-time cap stops the stage travelling more
        # than a safe fraction of the lookahead per control period → kills the
        # pure-pursuit overshoot limit-cycle ("back-and-forth").
        sl = getattr(ctrl, 'safety_limits', None)
        _fallback_max = 50000.0
        try:
            _m = float(getattr(sl, 'max_xy_speed', 0) or 0)
            if _m > 0:
                _fallback_max = _m
        except Exception:
            pass
        # v7.5.x: the full tuning dict (store's "velocity" bucket). Every key
        # defaults to 0 = legacy, so an uncalibrated machine is unchanged.
        try:
            _tune = settings.velocity_tuning()
        except Exception:
            _tune = {}

        def _tv(key, default=0.0):
            try:
                return float(_tune.get(key, default) or default)
            except (TypeError, ValueError):
                return default

        _res = _velctl.resolve_control(
            print_speed_mm_s=print_speed, lookahead_mm=_lookahead,
            xy_max_speed_um_s=getattr(settings, "xy_max_speed_um_s", 0.0),
            control_loop_ms=getattr(settings, "control_loop_ms", 0.0),
            phase_lag_s=getattr(settings, "phase_lag_s", 0.0),
            default_control_hz=self._VEL_CONTROL_HZ,
            fallback_max_um_s=_fallback_max,
            # Stage-1 speed decoupling — see VelocityControl.resolve_control.
            # dead_time_s (measured by XYDeadTime) supersedes phase_lag_s, which
            # is a settle time and on ME3B V1 caps prints at 0.31 mm/s.
            dead_time_s=_tv("dead_time_s"),
            lead_time_frac=_tv("lead_time_frac"),
            min_lookahead_frac=_tv("min_lookahead_frac"),
            max_speed_frac=_tv("max_speed_frac"),
            hold_speed=bool(_tv("hold_speed")),
            safety=(_tv("deadtime_safety") or 2.0))
        max_um_s = _res["max_um_s"]
        _ctrl_hz = _res["control_hz"]
        speed_cap = _res["speed_cap_mm_s"]
        # The lookahead may have been RESOLVED upward from the dynamics, so the
        # carrot must use the resolved value, not the requested one.
        _lookahead = _res.get("lookahead_mm", _lookahead)

        # Corner-aware speed-limit profile along the path (slow into sharp turns).
        speed_limit_at, _corners = _velctl.plan_speed_limits(
            pts, cum, print_speed, corner_angle_deg=_corner_ang,
            corner_speed_factor=_corner_fac, decel_mm=_decel)

        # Set SMS to the (measured) max so VS isn't capped below the commanded
        # velocity; brisk accel so VS actually reaches it.
        xy = getattr(ctrl, 'xy_stage', None)
        _jerk = _tv("jerk_pct") or float(getattr(settings, "xy_jerk_pct", 0.0) or 0.0)
        try:
            if xy is not None and hasattr(xy, 'set_acceleration'):
                xy.set_acceleration(
                    getattr(settings, 'xy_accel_pct', 80) or 80)
            if xy is not None and hasattr(xy, 'set_speed_mm_s'):
                xy.set_speed_mm_s(max_um_s / 1000.0)
            # Prior SCS S-curve limit — shapes corner overshoot. 0 = don't touch.
            # ⚠ changing it invalidates the measured loop period / dead time /
            # top speed, which is why it is logged with the run below.
            if _jerk > 0 and xy is not None and hasattr(xy, 'set_jerk'):
                xy.set_jerk(int(_jerk))
        except Exception:
            pass

        zero = getattr(ctrl, 'zero_position', {})

        def read_pos():
            try:
                p = ctrl.get_xy_position(cached=False)
            except Exception:
                return None
            if not p or p[0] is None or p[1] is None:
                return None
            return ((p[0] - zero.get('x', 0)) / 1000.0,
                    (p[1] - zero.get('y', 0)) / 1000.0)

        # Move to the path start (confirmed) before opening the loop.
        if lg:
            lg.log("xy_cmd", context="vel_path_start",
                   **lg.xy_cmd_fields(ctrl, pts[0][0], pts[0][1]))
        ctrl.move_xy_absolute(pts[0][0], pts[0][1], from_zero_ref=True)
        self._wait_for_xy_settle(pts[0][0], pts[0][1], timeout=5.0)

        if lg:
            lg.log("path_start", n_points=len(pts), pump=pump,
                   flow_rate_uL_s=flow_rate_uL_s, flow_rate=flow_rate,
                   speed_mm_s=round(print_speed, 3), mode="velocity",
                   n_corners=len(_corners), speed_cap=round(speed_cap, 3),
                   ctrl_hz=round(_ctrl_hz, 1), kp=_kp, kd=_kd,
                   # v7.5.x: make every run self-describing so a slow print can
                   # be diagnosed from its log alone — cap_reason names the term
                   # that is binding (print_speed / dead_time / top_speed /
                   # hold_speed), which is the question the operator kept asking.
                   lookahead_resolved=round(_lookahead, 4),
                   cap_reason=_res.get("cap_reason", ""),
                   dead_time_s=round(_res.get("dead_time_s", 0.0), 4),
                   lead_s=round(_res.get("lead_s", 0.0), 4),
                   jerk_pct=_jerk, tuning=dict(_tune),
                   **PrintExecutionLogger._path_stats(pts),
                   **PrintExecutionLogger.path_points(pts))

        dt = 1.0 / max(_ctrl_hz, 1.0)
        arrive_tol_mm = self._VEL_ARRIVE_TOL_UM / 1000.0
        state = _velctl.PursuitState()
        _t0 = time.monotonic()
        s_prev = 0.0
        pending_uL = 0.0
        last_pos_t = _t0
        runaway = 0
        n_ticks = 0
        max_wall = total / print_speed * 6.0 + 15.0

        try:
            while True:
                _tick = time.monotonic()

                if self._abort_flag.is_set():
                    if lg:
                        lg.log("path_end", mode="velocity", aborted=True,
                               s_mm=round(s_prev, 3),
                               wall_s=round(_tick - _t0, 3))
                    return
                # Mid-path ZP drop → stop (do not drag the needle dry).
                if (getattr(self, "_zp_connected_at_start", False)
                        and not getattr(ctrl, "is_zp_connected", True)):
                    logger.error("VEL_PATH: ZP disconnected mid-path — stopping")
                    if lg:
                        lg.log("path_end", mode="velocity", zp_disconnect=True,
                               s_mm=round(s_prev, 3),
                               wall_s=round(_tick - _t0, 3))
                    return
                if _tick - _t0 > max_wall:
                    logger.warning("VEL_PATH: wall-time cap hit at s=%.2f/%.2f mm",
                                   s_prev, total)
                    break

                pos = read_pos()
                if pos is None:
                    # No fresh position — hold last velocity briefly; stop if it
                    # persists (safety: never drive blind).
                    if _tick - last_pos_t > self._VEL_STALE_S:
                        ctrl.send_velocity_xy(0.0, 0.0)
                        if lg:
                            lg.log("vel_stall", s_mm=round(s_prev, 3),
                                   held_s=round(_tick - last_pos_t, 2))
                        # keep looping; a truly dead link trips ZP/abort paths
                    time.sleep(dt)
                    continue
                dt_real = max(1e-3, _tick - last_pos_t)
                last_pos_t = _tick

                # Bound the projection's forward search + the s advance to the
                # physically-plausible distance this tick, so a spiral/self-
                # intersecting path can't snap the projection to a later loop.
                max_ds = min(self._VEL_MAX_SNAP_MM,
                             max(0.15, print_speed * dt_real * self._VEL_SNAP_GUARD))

                # End-of-path decel folded into the per-tick speed cap (on top of
                # the resolved dead-time cap + corner-limit inside pursuit_step).
                remaining = total - s_prev
                _cap = speed_cap
                if remaining < _decel:
                    # ramp from the EFFECTIVE (possibly dead-time-capped) speed,
                    # not the raw commanded print_speed, so a capped run still
                    # slows to a clean stop at the endpoint (no endpoint dither).
                    _cap = min(_cap, max(0.1, remaining / _decel)
                               * min(print_speed, speed_cap))

                # One pure-pursuit + cross-track-PID tick (shared control law).
                vx, vy, s, cross = _velctl.pursuit_step(
                    pos, pts, cum, state, lookahead=_lookahead,
                    speed_cap_mm_s=_cap, speed_limit_at=speed_limit_at,
                    dt=dt_real, max_ds=max_ds, kp=_kp, kd=_kd)
                ds = s - s_prev

                # Clamp VS magnitude to the (measured) safety ceiling.
                vmag = math.hypot(vx, vy)
                if vmag > max_um_s and vmag > 0:
                    vx *= max_um_s / vmag
                    vy *= max_um_s / vmag

                # Runaway guard: sustained large perpendicular error ⇒ the stage
                # is not on the path (e.g. VS sign inverted) → abort safely.
                if cross > self._VEL_MAX_CROSS_TRACK_MM:
                    runaway += 1
                    if runaway >= self._VEL_RUNAWAY_TICKS:
                        ctrl.send_velocity_xy(0.0, 0.0)
                        raise RuntimeError(
                            f"velocity-follow runaway: cross-track "
                            f"{cross:.2f} mm > {self._VEL_MAX_CROSS_TRACK_MM} mm "
                            f"for {runaway} ticks (check VS direction / sign)")
                else:
                    runaway = 0

                # Deposit pump volume ∝ real distance travelled (non-blocking).
                if ds > 0:
                    if vol_per_mm > 0:
                        pending_uL += ds * vol_per_mm
                        if pending_uL > self._PATH_PUMP_EMIT_MIN_UL:
                            self._emit_pump(ctrl, pump, pending_uL,
                                            flow_rate_uL_s, settings)
                            pending_uL = 0.0
                    elif not use_uL and flow_rate:
                        self._emit_pump(ctrl, pump, ds * flow_rate, None, settings)

                # Arrived?
                dist_end = math.hypot(pos[0] - pts[-1][0], pos[1] - pts[-1][1])
                if s >= total - 1e-6 and dist_end <= arrive_tol_mm:
                    break

                ctrl.send_velocity_xy(vx, vy)

                n_ticks += 1
                if lg and (n_ticks % 5 == 0):
                    lg.log("vel_sample", s_mm=round(s, 3), tot_mm=round(total, 3),
                           cross_um=round(cross * 1000.0, 1),
                           x_mm=round(pos[0], 4), y_mm=round(pos[1], 4),
                           vx=round(vx, 0), vy=round(vy, 0))

                s_prev = s
                _elapsed = time.monotonic() - _tick
                if _elapsed < dt:
                    time.sleep(dt - _elapsed)
        finally:
            # ALWAYS stop the stage, however we leave the loop.
            try:
                ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass

        # Flush residual pump volume + confirm the stage has settled at the end.
        if pending_uL > 0 and vol_per_mm > 0:
            self._emit_pump(ctrl, pump, pending_uL, flow_rate_uL_s, settings)
        self._wait_for_xy_settle(pts[-1][0], pts[-1][1], timeout=10.0,
                                 tolerance=self._VEL_ARRIVE_TOL_UM)
        self._print_pump_suckback("deposit", pump)
        if lg:
            lg.log("path_end", mode="velocity",
                   wall_s=round(time.monotonic() - _t0, 3),
                   s_mm=round(min(s_prev, total), 3), tot_mm=round(total, 3))

    # ── v7.6: feature-aware FEED PLAN print path ───────────────────

    def _feed_plan_char(self, settings):
        """The machine's measured dynamics, from the STAMPED settings (not the
        global store) so the plan is reproducible from the job alone.

        Returns None — meaning "fall back to the legacy follower" — unless the
        machine is fully characterised (loop period + dead time + top speed).
        """
        try:
            from SupportClasses.XYStageModel import StageCharacteristics
        except Exception:                           # pragma: no cover
            return None
        try:
            tune = settings.velocity_tuning()
        except Exception:
            tune = {}

        def _f(v, default=0.0):
            try:
                return float(v or default)
            except (TypeError, ValueError):
                return default

        char = StageCharacteristics(
            dead_time_s=_f(tune.get("dead_time_s")),
            tau_s=_f(tune.get("tau_s")),
            top_speed_um_s=_f(getattr(settings, "xy_max_speed_um_s", 0.0)),
            control_loop_ms=_f(getattr(settings, "control_loop_ms", 0.0)))
        return char if char.is_complete() else None

    def _execute_print_path_feed_plan(self, cmd: PrintCommand) -> bool:
        """Print the path as a FEED PLAN: a sequence of feature-sized sections
        with a full stop on every sharp corner / reversal.

        Returns True when the path was printed here, False when the plan could
        not be built (**no motion issued** — the caller falls back to the legacy
        single-tuning follower and logs why).

        Why sections: one fixed tuning cannot hold a resolution element through
        a corner — pure pursuit cuts corners by ≈0.4·lookahead and cannot turn a
        180° reversal at all. Each section here gets its own curvature-sized
        lookahead and the speed that lookahead can sustain, and the plan stops
        ON each split vertex (tight arrive + the lag-aware end taper), so the
        corner error is the stage's own stopping accuracy (~8 µm) instead of a
        lookahead-scaled cut.

        Pump bookkeeping is GLOBAL across sections: deposition tracks the total
        arc length travelled, so the volume laid down is
        ``vol_per_mm × total_length`` regardless of where the splits fall, with
        nothing deposited during the inter-section stops.
        """
        from SupportClasses import XYFeedPlan as _fp

        ctrl = self.controller
        settings = self.job.settings
        raw_pts = cmd.params.get("points", [])
        pump = cmd.params.get("pump", self._active_pump)
        flow_rate_uL_s = cmd.params.get("flow_rate_uL_s", None)
        flow_rate = cmd.params.get("flow_rate", 0.01)
        use_uL = flow_rate_uL_s is not None
        lg = self.exec_logger

        if not raw_pts:
            return False
        pts = [(float(raw_pts[0][0]), float(raw_pts[0][1]))]
        for p in raw_pts[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
                pts.append((float(p[0]), float(p[1])))
        if len(pts) < 2:
            return False

        char = self._feed_plan_char(settings)
        if char is None:
            if lg:
                lg.log("feed_plan_fallback", reason="machine_not_characterised")
            logger.warning(
                "feed plan requested but the machine is not characterised "
                "(needs loop period + dead time + top speed) — using the "
                "legacy velocity follower")
            return False

        print_speed = getattr(settings, 'print_speed_mm_s', 0) or \
            max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        print_speed = max(0.05, float(print_speed))
        element_um = float(getattr(settings, "feed_plan_element_um", 0.0) or 0.0) \
            or 30.0
        split_deg = float(getattr(settings, "feed_plan_corner_split_deg", 0.0)
                          or 0.0) or _fp.CORNER_SPLIT_DEG

        # v7.7: "slow" is the default corner policy — slow through corners so the
        # pump never stops mid-path. "stop" keeps the v6 sectioned plan.
        policy = str(getattr(settings, "feed_plan_corner_policy", "slow")
                     or "slow").lower()
        if policy == "stop":
            plan = _fp.build_plan(pts, char, target_speed_mm_s=print_speed,
                                  element_um=element_um,
                                  corner_split_deg=split_deg)
        else:
            plan = _fp.build_continuous_plan(
                pts, char, target_speed_mm_s=print_speed,
                element_um=element_um)
        if not plan.sections:
            if lg:
                lg.log("feed_plan_fallback", reason="degenerate_plan")
            return False

        cum = polyline_arclength(pts)
        total = cum[-1]
        vol_per_mm = (flow_rate_uL_s / print_speed) if (
            use_uL and flow_rate_uL_s and flow_rate_uL_s > 0) else 0.0

        # SMS / accel / jerk exactly as the legacy path sets them.
        try:
            _tune = settings.velocity_tuning()
        except Exception:
            _tune = {}
        _jerk = float(_tune.get("jerk_pct", 0) or 0) or \
            float(getattr(settings, "xy_jerk_pct", 0.0) or 0.0)
        xy = getattr(ctrl, 'xy_stage', None)
        try:
            if xy is not None and hasattr(xy, 'set_acceleration'):
                xy.set_acceleration(getattr(settings, 'xy_accel_pct', 80) or 80)
            if xy is not None and hasattr(xy, 'set_speed_mm_s'):
                xy.set_speed_mm_s(char.top_speed_um_s / 1000.0)
            if _jerk > 0 and xy is not None and hasattr(xy, 'set_jerk'):
                xy.set_jerk(int(_jerk))
        except Exception:
            pass

        # Move to the path start (confirmed) before opening any loop.
        if lg:
            lg.log("xy_cmd", context="feed_plan_start",
                   **lg.xy_cmd_fields(ctrl, pts[0][0], pts[0][1]))
        ctrl.move_xy_absolute(pts[0][0], pts[0][1], from_zero_ref=True)
        self._wait_for_xy_settle(pts[0][0], pts[0][1], timeout=5.0)
        if self._abort_flag.is_set():
            return True                     # abort during the approach

        _t0 = time.monotonic()
        if lg:
            lg.log("path_start", n_points=len(pts), pump=pump,
                   flow_rate_uL_s=flow_rate_uL_s, flow_rate=flow_rate,
                   speed_mm_s=round(print_speed, 3), mode="velocity",
                   feed_plan=True, n_sections=len(plan.sections),
                   n_stops=plan.n_stops,
                   plan_est_s=round(plan.est_time_s, 1),
                   corner_policy=policy, continuous=bool(plan.continuous),
                   element_um=round(element_um, 1),
                   budget_um=round(plan.deviation_budget_um, 1),
                   jerk_pct=_jerk, tuning=dict(_tune),
                   **PrintExecutionLogger._path_stats(pts),
                   **PrintExecutionLogger.path_points(pts))

        # ── GLOBAL pump bookkeeping across sections ────────────────
        # base_s = arc length of all COMPLETED sections. Sections partition the
        # path and share their split vertices, so base_s + s_section is the
        # global progress; deposition on the monotone global delta can neither
        # double-count at a boundary nor deposit during a stop dwell.
        book = {"base_s": 0.0, "s_global": 0.0, "pending_uL": 0.0,
                # v7.7: cumulative deposited volume (never reset), reported live.
                "total_uL": 0.0}

        def deposit(s_sec: float, sec_len: float):
            """Advance the global volume bookkeeping. v7.7: returns the total µL
            deposited so far (or None when this path deposits nothing), so the
            live readout and the report can show dispensed-vs-planned without
            re-deriving it."""
            s_global = book["base_s"] + min(max(0.0, s_sec), sec_len)
            ds = s_global - book["s_global"]
            if ds <= 0:
                return book["total_uL"] if vol_per_mm > 0 else None
            book["s_global"] = s_global
            if vol_per_mm > 0:
                book["pending_uL"] += ds * vol_per_mm
                book["total_uL"] += ds * vol_per_mm
                if book["pending_uL"] > self._PATH_PUMP_EMIT_MIN_UL:
                    self._emit_pump(ctrl, pump, book["pending_uL"],
                                    flow_rate_uL_s, settings)
                    book["pending_uL"] = 0.0
                return book["total_uL"]
            elif not use_uL and flow_rate:
                self._emit_pump(ctrl, pump, ds * flow_rate, None, settings)
            return None

        n_done = 0
        status = "arrived"
        try:
            for k, sec in enumerate(plan.sections):
                if self._abort_flag.is_set():
                    status = "aborted"
                    break
                # A section shorter than its own arrive tolerance cannot be
                # "driven to" meaningfully — skip the motion, keep the volume
                # exact by advancing the global cursor.
                if sec.length_mm < max(2.0 * sec.arrive_mm, 0.02):
                    if lg:
                        lg.log("plan_section", index=k,
                               n_sections=len(plan.sections),
                               length_mm=round(sec.length_mm, 4),
                               skipped=True)
                    book["base_s"] += sec.length_mm
                    continue

                tuning = _fp.tuning_for(sec)
                resolved = _velctl.resolve_control(
                    print_speed_mm_s=sec.speed_mm_s,
                    lookahead_mm=sec.lookahead_mm,
                    xy_max_speed_um_s=char.top_speed_um_s,
                    control_loop_ms=char.control_loop_ms,
                    phase_lag_s=0.0,
                    default_control_hz=self._VEL_CONTROL_HZ,
                    fallback_max_um_s=char.top_speed_um_s or 50000.0,
                    dead_time_s=char.dead_time_s,
                    max_speed_frac=float(tuning.get("max_speed_frac", 0.9)),
                    hold_speed=True,
                    safety=float(tuning.get("deadtime_safety", 2.0)) or 2.0)
                end_lag_s, end_floor = _fp.section_end_taper(char, sec)
                if lg:
                    lg.log("plan_section", index=k,
                           n_sections=len(plan.sections),
                           lookahead_mm=round(sec.lookahead_mm, 4),
                           speed_mm_s=round(sec.speed_mm_s, 3),
                           length_mm=round(sec.length_mm, 4),
                           arrive_mm=sec.arrive_mm,
                           min_radius_mm=(round(sec.min_radius_mm, 3)
                                          if math.isfinite(sec.min_radius_mm)
                                          else None),
                           base_s_mm=round(book["base_s"], 4),
                           end_lag_s=round(end_lag_s, 4),
                           reason=sec.reason)

                status = self._run_plan_section(
                    sec, resolved=resolved, end_lag_s=end_lag_s,
                    end_floor_frac=end_floor, deposit=deposit,
                    sec_index=k, char=char)
                book["base_s"] += sec.length_mm
                book["s_global"] = max(book["s_global"], book["base_s"])
                n_done += 1
                if status != "arrived":
                    break

                # Inter-section STOP: the stage is already commanded to zero by
                # the section's own exit; dwell so the residual coast decays and
                # the next section starts from rest ON its start vertex. Chunked
                # so an abort exits within ~50 ms.
                if k < len(plan.sections) - 1:
                    try:
                        ctrl.send_velocity_xy(0.0, 0.0)
                    except Exception:
                        pass
                    _end = time.monotonic() + self._PLAN_STOP_DWELL_S
                    while time.monotonic() < _end:
                        if self._abort_flag.is_set():
                            status = "aborted"
                            break
                        time.sleep(0.05)
                    if status == "aborted":
                        break
        finally:
            try:
                ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass

        if status == "arrived":
            # Exactness top-up: fold any un-arrived residual (≤ the arrive
            # tolerance per section) into the final emission so the deposited
            # total is vol_per_mm × total for the plan path.
            rem = total - book["s_global"]
            if rem > 0 and vol_per_mm > 0:
                book["pending_uL"] += rem * vol_per_mm
        if book["pending_uL"] > 0 and vol_per_mm > 0:
            self._emit_pump(ctrl, pump, book["pending_uL"], flow_rate_uL_s,
                            settings)
            book["pending_uL"] = 0.0

        if status == "arrived":
            self._wait_for_xy_settle(pts[-1][0], pts[-1][1], timeout=10.0,
                                     tolerance=self._VEL_ARRIVE_TOL_UM)
            self._print_pump_suckback("deposit", pump)
        if lg:
            lg.log("path_end", mode="velocity", feed_plan=True,
                   status=status, aborted=(status == "aborted"),
                   sections_done=n_done, n_sections=len(plan.sections),
                   wall_s=round(time.monotonic() - _t0, 3),
                   s_mm=round(min(book["s_global"], total), 3),
                   tot_mm=round(total, 3))
        return True

    #: Settle dwell at an inter-section stop (s). Long enough for the residual
    #: coast to decay so the next section starts from rest.
    _PLAN_STOP_DWELL_S = 0.3

    def _run_plan_section(self, sec, *, resolved, end_lag_s, end_floor_frac,
                          deposit, sec_index, char) -> str:
        """Drive ONE feed-plan section. Returns ``"arrived"`` / ``"aborted"`` /
        ``"zp_disconnect"`` / ``"wall_cap"``.

        The legacy per-tick body, parametrized per section: the section's own
        lookahead + speed cap + arrive tolerance, a fresh ``PursuitState``, and
        the lag-aware end taper that lands the stop ON the vertex. Every legacy
        guard is retained (abort per tick, mid-path ZP drop, stale position,
        bounded projection window, runaway → RuntimeError, VS clamp).
        """
        ctrl = self.controller
        settings = self.job.settings
        lg = self.exec_logger
        pts = sec.pts
        cum = polyline_arclength(pts)
        total = cum[-1]
        speed = max(0.05, float(sec.speed_mm_s))
        speed_cap = resolved["speed_cap_mm_s"]
        lookahead = resolved.get("lookahead_mm", sec.lookahead_mm)
        max_um_s = resolved["max_um_s"]
        dt = 1.0 / max(resolved["control_hz"], 1.0)
        decel = max(0.15, min(0.5, sec.length_mm * 0.3))
        arrive_mm = float(sec.arrive_mm)

        # v7.7: a CONTINUOUS section carries arc-length profiles — corners inside
        # it are slowed through, so the carrot and the speed both vary with s.
        # A sectioned ("stop") plan has no sharp corners by construction, so its
        # corner profile is a deliberate no-op (pursuit_step always calls it).
        lookahead_at = getattr(sec, "lookahead_at", None)
        speed_limit_at = getattr(sec, "speed_at", None)
        if speed_limit_at is None:
            speed_limit_at, _corners = _velctl.plan_speed_limits(
                pts, cum, speed, corner_angle_deg=89.0, corner_speed_factor=1.0,
                decel_mm=decel)
        # The wall cap must be sized on the SLOWEST commanded point, or a corner
        # crawl trips it. Kept separate from `speed`, which the end taper uses.
        v_wall = speed
        if speed_limit_at is not None and hasattr(speed_limit_at, "min_value"):
            try:
                v_wall = max(0.05, float(speed_limit_at.min_value()))
            except Exception:
                v_wall = speed

        zero = getattr(ctrl, 'zero_position', {})

        def read_pos():
            try:
                p = ctrl.get_xy_position(cached=False)
            except Exception:
                return None
            if not p or p[0] is None or p[1] is None:
                return None
            return ((p[0] - zero.get('x', 0)) / 1000.0,
                    (p[1] - zero.get('y', 0)) / 1000.0)

        state = _velctl.PursuitState()
        _t0 = time.monotonic()
        last_pos_t = _t0
        prev_pos = None
        v_meas = 0.0
        runaway = 0
        n_ticks = 0
        s_prev = 0.0
        max_wall = sec.length_mm / v_wall * 8.0 + 10.0
        try:
            while True:
                _tick = time.monotonic()
                if self._abort_flag.is_set():
                    return "aborted"
                if (getattr(self, "_zp_connected_at_start", False)
                        and not getattr(ctrl, "is_zp_connected", True)):
                    logger.error("FEED_PLAN: ZP disconnected mid-path — stopping")
                    return "zp_disconnect"
                if _tick - _t0 > max_wall:
                    logger.warning(
                        "FEED_PLAN section %d: wall-time cap at s=%.2f/%.2f mm",
                        sec_index, s_prev, total)
                    return "wall_cap"

                pos = read_pos()
                if pos is None:
                    if _tick - last_pos_t > self._VEL_STALE_S:
                        ctrl.send_velocity_xy(0.0, 0.0)
                        if lg:
                            lg.log("vel_stall", sec=sec_index,
                                   s_mm=round(s_prev, 3),
                                   held_s=round(_tick - last_pos_t, 2))
                    time.sleep(dt)
                    continue
                dt_real = max(1e-3, _tick - last_pos_t)
                last_pos_t = _tick
                if prev_pos is not None:
                    v_meas = math.hypot(pos[0] - prev_pos[0],
                                        pos[1] - prev_pos[1]) / dt_real
                prev_pos = pos

                max_ds = min(self._VEL_MAX_SNAP_MM,
                             max(0.15, speed * dt_real * self._VEL_SNAP_GUARD))

                # LAG-AWARE end taper: brake for where the stage WILL be when
                # this command takes effect, using the MEASURED speed (during
                # braking the actual speed exceeds the commanded one, so the
                # commanded value under-predicts the flight distance). Without
                # this the stop overshoots the vertex by ≈ v·lag (153 µm at
                # 3 mm/s on ME3B V1) and the corner blows the element budget.
                remaining = total - s_prev
                _cap = speed_cap
                if remaining < decel:
                    rem_eff = remaining
                    if end_lag_s > 0.0 and v_meas > 0.0:
                        rem_eff = max(0.0, remaining - v_meas * end_lag_s)
                    _cap = min(_cap, max(end_floor_frac, rem_eff / decel)
                               * min(speed, speed_cap))

                vx, vy, s, cross = _velctl.pursuit_step(
                    pos, pts, cum, state, lookahead=lookahead,
                    speed_cap_mm_s=_cap, speed_limit_at=speed_limit_at,
                    dt=dt_real, max_ds=max_ds, kp=0.0, kd=0.0,
                    lookahead_at=lookahead_at)

                vmag = math.hypot(vx, vy)
                if vmag > max_um_s and vmag > 0:
                    vx *= max_um_s / vmag
                    vy *= max_um_s / vmag

                if cross > self._VEL_MAX_CROSS_TRACK_MM:
                    runaway += 1
                    if runaway >= self._VEL_RUNAWAY_TICKS:
                        ctrl.send_velocity_xy(0.0, 0.0)
                        raise RuntimeError(
                            f"feed-plan runaway: cross-track {cross:.2f} mm > "
                            f"{self._VEL_MAX_CROSS_TRACK_MM} mm for {runaway} "
                            f"ticks (check VS direction / sign)")
                else:
                    runaway = 0

                _dep_uL = deposit(s, sec.length_mm)

                dist_end = math.hypot(pos[0] - pts[-1][0], pos[1] - pts[-1][1])
                if s >= total - 1e-6 and dist_end <= arrive_mm:
                    return "arrived"

                ctrl.send_velocity_xy(vx, vy)

                n_ticks += 1
                # v7.7: the GUI's live readout and the post-print report both
                # hang off THIS existing every-5th-tick branch (~5 Hz), so the
                # ~25 Hz control loop gains no per-tick work. `v_meas` was
                # already computed above and thrown away; `deposited_uL` is the
                # executor's own global volume bookkeeping.
                if n_ticks % 5 == 0 and (lg or self.on_vel_sample):
                    _rec = {"sec": sec_index,
                            "s_mm": round(s, 3), "tot_mm": round(total, 3),
                            "cross_um": round(cross * 1000.0, 1),
                            "x_mm": round(pos[0], 4), "y_mm": round(pos[1], 4),
                            "vx": round(vx, 0), "vy": round(vy, 0),
                            "v_meas_mm_s": round(v_meas, 4),
                            "deposited_uL": (None if _dep_uL is None
                                             else round(_dep_uL, 5))}
                    if lg:
                        lg.log("vel_sample", **_rec)
                    if self.on_vel_sample is not None:
                        try:
                            self.on_vel_sample(_rec)
                        except Exception as _e:      # never break the loop
                            logger.debug("on_vel_sample failed: %s", _e)

                s_prev = s
                _elapsed = time.monotonic() - _tick
                if _elapsed < dt:
                    time.sleep(dt - _elapsed)
        finally:
            try:
                ctrl.send_velocity_xy(0.0, 0.0)
            except Exception:
                pass

    def _emit_pump(self, ctrl, pump, uL, flow_rate_uL_s, settings):
        """Emit a single (non-blocking) pump dispense of ``uL`` µL for the
        velocity follower — µL path when available, else the legacy axis-map
        relative move. Never blocks the control loop (no M400)."""
        if uL <= 0:
            return
        try:
            if hasattr(ctrl, 'move_pump_uL'):
                ctrl.move_pump_uL(pump, uL, flow_rate_uL_s)
            else:
                _axis_map = getattr(ctrl.zp_stage, 'axis_map', AXIS_MAP) \
                    if getattr(ctrl, 'zp_stage', None) else AXIS_MAP
                mapped = _axis_map.get(pump, AXIS_MAP.get(pump))
                if mapped and getattr(ctrl, 'zp_stage', None):
                    ctrl.zp_stage.move_relative(
                        {mapped: uL * 0.3}, settings.pump_feedrate)
        except Exception as e:
            logger.debug(f"velocity pump emit skipped: {e}")

    def _wait_for_xy_settle(self, target_x, target_y, timeout=3.0, tolerance=50):
        """
        Wait for XY stage to reach target position.

        v7.2.7: mm-based settle — shorter timeout, better logging.
        target_x/y are in mm (zero-ref). tolerance in µm.
        """
        ctrl = self.controller
        if not hasattr(ctrl, 'xy_stage') or not ctrl.xy_stage:
            time.sleep(0.1)
            return

        # Convert target mm → µm for comparison with stage position
        target_x_um = target_x * 1000.0 + ctrl.zero_position.get("x", 0)
        target_y_um = target_y * 1000.0 + ctrl.zero_position.get("y", 0)

        lg = self.exec_logger
        last_err_um = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._abort_flag.is_set():
                if lg:
                    lg.log("settle_wait", ok=False, reason="aborted",
                           target_x_mm=round(target_x, 4),
                           target_y_mm=round(target_y, 4),
                           duration_s=round(time.monotonic() - t0, 3))
                return
            pos = ctrl.get_xy_position(cached=False)
            if pos[0] is not None:
                dx = abs(pos[0] - target_x_um)
                dy = abs(pos[1] - target_y_um)
                last_err_um = math.hypot(dx, dy)
                if dx < tolerance and dy < tolerance:
                    if lg:
                        lg.log("settle_wait", ok=True,
                               target_x_mm=round(target_x, 4),
                               target_y_mm=round(target_y, 4),
                               duration_s=round(time.monotonic() - t0, 3),
                               final_err_um=round(last_err_um, 1))
                    return
            time.sleep(0.05)

        logger.debug(f"v7.2.7: Settle timeout after {timeout}s "
                     f"(target={target_x:.2f},{target_y:.2f}mm)")
        # v7.5.x exec log: a settle TIMEOUT silently continues execution —
        # capture it; this is a prime suspect for desync bugs.
        if lg:
            lg.log("settle_wait", ok=False, reason="timeout",
                   target_x_mm=round(target_x, 4),
                   target_y_mm=round(target_y, 4),
                   duration_s=round(time.monotonic() - t0, 3),
                   timeout_s=timeout,
                   final_err_um=(round(last_err_um, 1)
                                 if last_err_um is not None else None))


class PrintQueue:
    """
    Manages a queue of PrintJobs and executes them sequentially.
    
    Usage:
        queue = PrintQueue(controller)
        queue.add_job(job1)
        queue.add_job(job2)
        queue.on_progress = callback  # per-job progress
        queue.on_queue_progress = callback  # overall queue progress
        queue.start_all()
    """

    def __init__(self, controller):
        self.controller = controller
        self.queue: list[PrintJob] = []
        self.current_index: int = -1
        self.print_manager = PrintManager(controller)

        # Callbacks
        self.on_progress: Optional[Callable] = None          # (step, total, msg) per-job
        self.on_queue_progress: Optional[Callable] = None     # (job_idx, total_jobs, job_name)
        self.on_state_changed: Optional[Callable] = None      # (PrintState)
        self.on_queue_completed: Optional[Callable] = None    # ()

        self._running = False
        self._abort_flag = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def total_jobs(self) -> int:
        return len(self.queue)

    @property
    def is_running(self) -> bool:
        return self._running

    def add_job(self, job: PrintJob):
        """Add a job to the queue."""
        self.queue.append(job)
        logger.info(f"Job added to queue: {job.name} (queue size: {len(self.queue)})")

    def remove_job(self, index: int):
        """Remove a job from the queue by index."""
        if 0 <= index < len(self.queue):
            removed = self.queue.pop(index)
            logger.info(f"Job removed from queue: {removed.name}")

    def reorder(self, from_idx: int, to_idx: int):
        """Move a job from one position to another."""
        if 0 <= from_idx < len(self.queue) and 0 <= to_idx < len(self.queue):
            job = self.queue.pop(from_idx)
            self.queue.insert(to_idx, job)

    def clear(self):
        """Clear all jobs from the queue."""
        self.queue.clear()
        self.current_index = -1
        logger.info("Print queue cleared")

    def start_all(self):
        """Start executing all jobs in the queue sequentially."""
        if self._running:
            logger.warning("Queue already running")
            return
        if not self.queue:
            logger.warning("Queue is empty")
            return

        self._abort_flag.clear()
        self._running = True
        self._thread = threading.Thread(target=self._run_queue, daemon=True)
        self._thread.start()

    def abort(self):
        """Abort the entire queue."""
        self._abort_flag.set()
        self.print_manager.abort()
        self._running = False

    def abort_current(self):
        """Abort only the current job, continue with next."""
        self.print_manager.abort()

    def pause(self):
        """Pause the current job."""
        self.print_manager.pause()

    def resume(self):
        """Resume the current job."""
        self.print_manager.resume()

    def get_total_progress(self) -> tuple[int, int]:
        """Get total progress across all jobs (completed_steps, total_steps)."""
        total = sum(j.total_steps for j in self.queue)
        completed = sum(j.total_steps for j in self.queue[:self.current_index])
        if 0 <= self.current_index < len(self.queue):
            completed += self.print_manager._current_step
        return completed, total

    def _run_queue(self):
        """Execute all jobs in sequence."""
        for i, job in enumerate(self.queue):
            if self._abort_flag.is_set():
                break

            self.current_index = i
            logger.info(f"Queue: starting job {i + 1}/{len(self.queue)}: {job.name}")

            if self.on_queue_progress:
                self.on_queue_progress(i + 1, len(self.queue), job.name)

            # Wire callbacks
            self.print_manager.on_progress = self.on_progress
            self.print_manager.on_state_changed = self.on_state_changed

            self.print_manager.load_job(job)
            self.print_manager.start()

            # Wait for job to finish
            while self.print_manager.state == PrintState.RUNNING or \
                  self.print_manager.state == PrintState.PAUSED:
                if self._abort_flag.is_set():
                    self.print_manager.abort()
                    break
                time.sleep(0.2)

            if self.print_manager.state == PrintState.ERROR:
                logger.error(f"Queue: job {job.name} failed, stopping queue")
                break

        self._running = False
        self.current_index = -1

        if self.on_queue_completed:
            self.on_queue_completed()

        logger.info("Print queue finished")


# ═══════════════════════════════════════════════════════════════════
# Utility: Save print job to JSON
# ═══════════════════════════════════════════════════════════════════

def save_print_job(job: PrintJob, filepath: str):
    """Save a PrintJob to JSON file."""
    data = {
        "version": "7.2",
        "name": job.name,
        "description": job.description,
        "settings": {
            field_name: getattr(job.settings, field_name)
            for field_name in job.settings.__dataclass_fields__
        },
        "commands": [
            {
                "type": cmd.type.value,
                "label": cmd.label,
                **cmd.params,
            }
            for cmd in job.commands
        ],
    }

    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)
    logger.info(f"Saved print job to {filepath}")


# ═══════════════════════════════════════════════════════════════════
# Enhancement 1: G-code Export
# ═══════════════════════════════════════════════════════════════════

def export_gcode(job: PrintJob, filepath: str) -> None:
    """
    Export a PrintJob to G-code format for compatibility with external slicers.

    Maps internal command types to standard G-code:
        MOVE_XY    → G0 Xn Yn
        MOVE_Z     → G0 Zn
        MOVE_Z_REL → G91; G0 Zn; G90
        DISPENSE   → G1 En Fn
        PRINT_PATH → G1 Xn Yn En Fn (coordinated moves)
        DWELL      → G4 Sn
        TRAVEL_UP  → G0 Z{travel_height}
        TRAVEL_DOWN→ G0 Z{print_height}
        HOME_XY    → G28 X Y
        COMMENT    → ; comment text
        SWITCH_PUMP→ ; SWITCH_PUMP comment (no G-code equivalent)

    Args:
        job: PrintJob to export.
        filepath: Output .gcode file path.
    """
    path = Path(filepath)
    settings = job.settings
    lines: list[str] = []

    # Header
    lines.append(f"; G-code exported from MEBP v7.2")
    lines.append(f"; Job: {job.name}")
    lines.append(f"; Description: {job.description}")
    lines.append(f"; Generated: {__import__('datetime').datetime.now().isoformat()}")
    lines.append(f"; Commands: {job.total_steps}")
    lines.append(f";")
    lines.append(f"; Print Settings:")
    lines.append(f";   XY feedrate: {settings.xy_feedrate}")
    lines.append(f";   Z feedrate: {settings.z_feedrate}")
    lines.append(f";   Print feedrate: {settings.print_feedrate}")
    lines.append(f";   Travel Z: {settings.travel_z_height} mm")
    lines.append(f";   Print Z: {settings.print_z_height} mm")
    lines.append(f";   Layers: {settings.num_layers}")
    lines.append(f";   Layer height: {settings.layer_height} mm")
    lines.append("")

    # Initialization
    lines.append("; Initialization")
    lines.append("G90 ; Absolute positioning")
    lines.append("G21 ; Millimeters")
    lines.append("M302 S0 ; Allow cold extrusion")
    lines.append("M83 ; Relative extrusion")
    lines.append(f"G0 F{settings.xy_feedrate} ; Set travel feedrate")
    lines.append("")

    active_pump_idx = 0  # Track pump for E-axis mapping

    for i, cmd in enumerate(job.commands):
        p = cmd.params

        if cmd.type == CommandType.COMMENT:
            lines.append(f"; {cmd.label}")

        elif cmd.type == CommandType.MOVE_XY:
            x, y = p.get("x", 0), p.get("y", 0)
            lines.append(f"G0 X{x:.4f} Y{y:.4f} F{settings.xy_feedrate} ; {cmd.label}")

        elif cmd.type == CommandType.MOVE_Z:
            z = p.get("z", 0)
            lines.append(f"G0 Z{z:.4f} F{settings.z_feedrate} ; {cmd.label}")

        elif cmd.type == CommandType.MOVE_Z_REL:
            dist = p.get("distance", 0)
            feedrate = p.get("feedrate", settings.z_feedrate)
            lines.append("G91 ; Relative")
            lines.append(f"G0 Z{dist:.4f} F{feedrate}")
            lines.append("G90 ; Absolute")

        elif cmd.type == CommandType.DISPENSE:
            pump = p.get("pump", "P1")
            if "amount_uL" in p:
                # v7.2: µL-based — note in comment, use raw value for G-code
                amount_uL = p["amount_uL"]
                rate = p.get("rate_uL_s", settings.pump_rate_uL_s if hasattr(settings, 'pump_rate_uL_s') else 0.25)
                lines.append(f"; v7.2 DISPENSE {pump}: {amount_uL:.3f} µL at {rate:.3f} µL/s")
                lines.append(f"G1 E{amount_uL:.5f} F{rate * 60:.1f} ; {pump} {cmd.label} (µL units)")
            else:
                amount = p.get("amount", 0)
                feedrate = p.get("feedrate", settings.pump_feedrate)
                lines.append(f"G1 E{amount:.5f} F{feedrate} ; {pump} {cmd.label}")

        elif cmd.type == CommandType.PRINT_PATH:
            points = p.get("points", [])
            flow_rate = p.get("flow_rate", 0.01)
            pump = p.get("pump", "P1")
            lines.append(f"; Print path ({len(points)} points, {pump}, flow={flow_rate})")
            if len(points) >= 2:
                # Move to first point
                lines.append(f"G0 X{points[0][0]:.4f} Y{points[0][1]:.4f} F{settings.xy_feedrate}")
                # Print segments with extrusion
                for j in range(1, len(points)):
                    x1, y1 = points[j - 1][0], points[j - 1][1]
                    x2, y2 = points[j][0], points[j][1]
                    seg_len = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    extrude = seg_len * flow_rate
                    lines.append(
                        f"G1 X{x2:.4f} Y{y2:.4f} E{extrude:.5f} F{settings.print_feedrate}"
                    )

        elif cmd.type == CommandType.DWELL:
            seconds = p.get("seconds", 0)
            lines.append(f"G4 S{seconds:.1f} ; Dwell")

        elif cmd.type == CommandType.TRAVEL_UP:
            lines.append(f"G0 Z{settings.travel_z_height:.4f} F{settings.z_feedrate} ; Travel up")

        elif cmd.type == CommandType.TRAVEL_DOWN:
            lines.append(f"G0 Z{settings.print_z_height:.4f} F{settings.z_feedrate} ; Travel down")

        elif cmd.type == CommandType.HOME_XY:
            lines.append("G0 X0 Y0 ; Home XY")

        elif cmd.type == CommandType.SET_PUMP_RATE:
            lines.append(f"; Set pump rate (no G-code equivalent)")

        elif cmd.type == CommandType.SWITCH_PUMP:
            new_pump = p.get("pump", "P1")
            old_pump = p.get("old_pump", "P1")
            retract_amt = settings.get_retract_amount(old_pump)
            prime_amt = settings.get_prime_amount(new_pump)
            lines.append(f"; SWITCH_PUMP: {old_pump} → {new_pump}")
            if retract_amt > 0:
                lines.append(f"G1 E{-retract_amt:.5f} F{settings.pump_feedrate} ; Retract {old_pump}")
            if prime_amt > 0:
                lines.append(f"G1 E{prime_amt:.5f} F{settings.pump_feedrate} ; Prime {new_pump}")

    # Footer
    lines.append("")
    lines.append("; End of print")
    lines.append(f"G0 Z{settings.travel_z_height:.4f} ; Final travel up")
    lines.append("G0 X0 Y0 ; Return home")
    lines.append("M84 ; Motors off")

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Exported G-code to {path} ({len(lines)} lines)")


# ═══════════════════════════════════════════════════════════════════
# Enhancement 3: Print Resume — Save/Load Progress
# ═══════════════════════════════════════════════════════════════════

_RESUME_FILE = "print_resume.json"


def save_print_progress(
    job: PrintJob,
    current_step: int,
    active_pump: str = "P1",
    filepath: str = _RESUME_FILE,
) -> None:
    """
    Save print progress to allow resuming after crash/disconnect.

    Saves the full job definition plus current execution state.
    Called periodically during printing and on pause.
    """
    data = {
        "version": 1,
        "saved_at": __import__("datetime").datetime.now().isoformat(),
        "current_step": current_step,
        "active_pump": active_pump,
        "job": {
            "name": job.name,
            "description": job.description,
            "source_file": job.source_file,
            "settings": {
                fname: getattr(job.settings, fname)
                for fname in job.settings.__dataclass_fields__
            },
            "commands": [
                {"type": cmd.type.value, "label": cmd.label, **cmd.params}
                for cmd in job.commands
            ],
        },
    }
    try:
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        logger.debug(f"Print progress saved at step {current_step}/{job.total_steps}")
    except Exception as e:
        logger.error(f"Failed to save print progress: {e}")


def load_print_progress(filepath: str = _RESUME_FILE) -> dict | None:
    """
    Load saved print progress for resume.

    Returns:
        Dict with 'job' (PrintJob), 'current_step', 'active_pump'
        or None if no resume file exists.
    """
    path = Path(filepath)
    if not path.exists():
        return None
    try:
        with open(path) as f:
            data = json.load(f)

        # Reconstruct the job
        job_data = data["job"]
        settings = PrintSettings.from_dict(job_data.get("settings", {}))
        commands = []
        for cmd_data in job_data.get("commands", []):
            cmd_type_str = cmd_data.get("type", "")
            try:
                cmd_type = CommandType(cmd_type_str)
            except ValueError:
                continue
            params = {k: v for k, v in cmd_data.items() if k != "type"}
            label = params.pop("label", "")
            commands.append(PrintCommand(type=cmd_type, params=params, label=label))

        job = PrintJob(
            name=job_data.get("name", "Resumed Job"),
            description=job_data.get("description", ""),
            settings=settings,
            commands=commands,
            source_file=job_data.get("source_file", ""),
        )

        result = {
            "job": job,
            "current_step": data.get("current_step", 0),
            "active_pump": data.get("active_pump", "P1"),
            "saved_at": data.get("saved_at", ""),
        }
        logger.info(
            f"Resume data loaded: {job.name} at step "
            f"{result['current_step']}/{job.total_steps}"
        )
        return result

    except Exception as e:
        logger.error(f"Failed to load print progress: {e}")
        return None


def clear_print_progress(filepath: str = _RESUME_FILE) -> None:
    """Remove the resume file after successful completion."""
    path = Path(filepath)
    if path.exists():
        try:
            path.unlink()
            logger.debug("Print resume file cleared")
        except Exception as e:
            logger.warning(f"Failed to clear resume file: {e}")


# ═══════════════════════════════════════════════════════════════════
# v7.2: Print File Version Migration
# ═══════════════════════════════════════════════════════════════════

def migrate_print_file_v71_to_v72(data: dict, hardware_config=None) -> dict:
    """
    Migrate a v7.1 print file to v7.2 format.

    Converts mm-based pump amounts to µL using the hardware config.
    If no hardware config is available, cannot convert and returns
    the data with a migration warning flag.

    Args:
        data: Parsed JSON print file dict
        hardware_config: HardwareConfig with syringe specs for conversion

    Returns:
        Migrated data dict with v7.2 format commands
    """
    file_version = data.get("version", "7.1")
    if file_version >= "7.2":
        return data

    logger.info(f"Migrating print file from v{file_version} to v7.2")

    migrated = dict(data)
    migrated["version"] = "7.2"
    migrated["migrated_from"] = file_version

    if not hardware_config:
        logger.warning(
            "No hardware config — cannot convert mm→µL. "
            "Commands with legacy 'amount' (mm) will be executed as-is."
        )
        migrated["_migration_incomplete"] = True
        return migrated

    commands = migrated.get("commands", [])
    converted_count = 0

    for cmd in commands:
        cmd_type = cmd.get("type", "")

        if cmd_type == "extrude" and "amount" in cmd and "amount_uL" not in cmd:
            pump = cmd.get("pump", "P1")
            amount_mm = cmd["amount"]
            pump_cfg = hardware_config.pumps.get(pump)

            if pump_cfg and pump_cfg.is_configured:
                try:
                    cmd["amount_uL"] = pump_cfg.mm_to_uL(amount_mm)
                    if "feedrate" in cmd:
                        cmd["rate_uL_s"] = pump_cfg.feedrate_mm_min_to_uL_s(
                            cmd["feedrate"]
                        )
                    # Archive legacy values
                    cmd["_legacy_amount_mm"] = cmd.pop("amount")
                    if "feedrate" in cmd:
                        cmd["_legacy_feedrate_mm_min"] = cmd.pop("feedrate")
                    converted_count += 1
                except (ValueError, AttributeError) as e:
                    logger.warning(f"Migration failed for {pump} extrude: {e}")

        elif cmd_type == "print_path" and "flow_rate" in cmd and "flow_rate_uL_s" not in cmd:
            # Legacy flow_rate was dimensionless ratio — flag for manual review
            cmd["flow_rate_uL_s"] = cmd.get("flow_rate", 0.01)
            cmd["_legacy_flow_rate"] = cmd["flow_rate"]
            cmd["_needs_review"] = True
            converted_count += 1

    # Migrate settings
    settings = migrated.get("settings", {})
    if "pump_feedrate" in settings and "pump_rate_uL_s" not in settings:
        settings["pump_rate_uL_s"] = 0.25  # Default — needs manual verification

    if "retract_amounts" in settings and "retract_amounts_uL" not in settings:
        settings["retract_amounts_uL"] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    if "prime_amounts" in settings and "prime_amounts_uL" not in settings:
        settings["prime_amounts_uL"] = {"P1": 0.0, "P2": 0.0, "P3": 0.0}

    logger.info(f"Migration complete: {converted_count} commands converted")
    return migrated


def detect_print_file_version(data: dict) -> str:
    """
    Detect the version of a print file.

    Returns "7.2" if any command has amount_uL or flow_rate_uL_s,
    otherwise returns whatever is in the version field or "7.1".
    """
    # Explicit version tag
    if "version" in data:
        return str(data["version"])

    # Heuristic: check commands for v7.2 fields
    for cmd in data.get("commands", []):
        if "amount_uL" in cmd or "flow_rate_uL_s" in cmd or "rate_uL_s" in cmd:
            return "7.2"

    # Heuristic: check settings for v7.2 fields
    settings = data.get("settings", {})
    if "pump_rate_uL_s" in settings or "retract_amounts_uL" in settings:
        return "7.2"

    return "7.1"

