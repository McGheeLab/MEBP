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
    EXTRUDE = "extrude"             # Extrude from a pump (relative)
    PRINT_PATH = "print_path"       # Coordinated XY move + extrusion
    DWELL = "dwell"                 # Wait for specified time
    TRAVEL_UP = "travel_up"         # Raise Z to travel height
    TRAVEL_DOWN = "travel_down"     # Lower Z to print height
    SET_PUMP_RATE = "set_pump_rate" # Set pump flow rate for upcoming moves
    HOME_XY = "home_xy"             # Move XY to zero reference
    COMMENT = "comment"             # No-op, just a label/comment
    SWITCH_PUMP = "switch_pump"     # Session 4: Switch active pump (multi-material)


@dataclass
class PrintCommand:
    """A single command in a print job."""
    type: CommandType
    params: dict = field(default_factory=dict)
    label: str = ""  # Human-readable description

    def __repr__(self):
        return f"PrintCommand({self.type.value}, {self.params}, '{self.label}')"


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
    retract_amount: float = 0.0      # Pump retraction after path segment (legacy single-pump)
    prime_amount: float = 0.0        # Pump prime before path segment (legacy single-pump)
    dwell_after_move: float = 0.0    # Seconds to wait after travel moves

    # Session 4: Per-pump retract/prime amounts
    retract_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})
    prime_amounts: dict = field(default_factory=lambda: {"P1": 0.0, "P2": 0.0, "P3": 0.0})

    @classmethod
    def from_dict(cls, d: dict) -> "PrintSettings":
        """Create from dictionary, ignoring unknown keys."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in d.items() if k in valid_keys}
        return cls(**filtered)

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
    """Load from custom JSON format."""
    with open(path, "r") as f:
        data = json.load(f)

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
        
    Returns:
        PrintJob ready for execution
    """
    commands = []
    active_pump = pump

    for layer in range(settings.num_layers):
        z_height = settings.print_z_height + layer * settings.layer_height

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

            # Travel to well
            commands.append(PrintCommand(
                type=CommandType.TRAVEL_UP,
                label=f"Travel up for {well_name}",
            ))
            commands.append(PrintCommand(
                type=CommandType.MOVE_XY,
                params={"x": well_x + path_points[0][0], "y": well_y + path_points[0][1]},
                label=f"Move to {well_name} start",
            ))

            if settings.dwell_after_move > 0:
                commands.append(PrintCommand(
                    type=CommandType.DWELL,
                    params={"seconds": settings.dwell_after_move},
                    label="Settle",
                ))

            # Lower to print height
            commands.append(PrintCommand(
                type=CommandType.MOVE_Z,
                params={"z": z_height},
                label="Lower to print height",
            ))

            # Prime (using per-pump amount)
            prime_amt = settings.get_prime_amount(active_pump)
            if prime_amt > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": prime_amt,
                            "feedrate": settings.pump_feedrate},
                    label=f"Prime {active_pump}",
                ))

            # Print the path in this well
            well_path = [(well_x + px, well_y + py) for px, py in path_points]
            commands.append(PrintCommand(
                type=CommandType.PRINT_PATH,
                params={
                    "points": well_path,
                    "pump": active_pump,
                    "flow_rate": flow_rate,
                },
                label=f"Print in {well_name}",
            ))

            # Retract (using per-pump amount)
            retract_amt = settings.get_retract_amount(active_pump)
            if retract_amt > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": active_pump, "amount": -retract_amt,
                            "feedrate": settings.pump_feedrate},
                    label=f"Retract {active_pump}",
                ))

    # Final travel up
    commands.append(PrintCommand(
        type=CommandType.TRAVEL_UP,
        label="Final travel up",
    ))
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
    """

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

        # Thread control
        self._thread: Optional[threading.Thread] = None
        self._pause_event = threading.Event()
        self._pause_event.set()  # Not paused initially
        self._abort_flag = threading.Event()

        # Current step tracking
        self._current_step = 0

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

        self._thread = threading.Thread(target=self._execute_loop, daemon=True)
        self._thread.start()
        self._set_state(PrintState.RUNNING)

    def pause(self):
        """Pause the current print. Completes the current command first."""
        if self.state != PrintState.RUNNING:
            return
        self._pause_event.clear()
        self._set_state(PrintState.PAUSED)
        logger.info("Print paused")

    def resume(self):
        """Resume a paused print."""
        if self.state != PrintState.PAUSED:
            return
        self._pause_event.set()
        self._set_state(PrintState.RUNNING)
        logger.info("Print resumed")

    def abort(self):
        """Abort the current print. Raises Z to travel height."""
        if self.state not in (PrintState.RUNNING, PrintState.PAUSED):
            return
        self._abort_flag.set()
        self._pause_event.set()  # Unblock if paused
        self._set_state(PrintState.ABORTED)
        logger.info("Print aborted")

        # Safety: raise Z to travel height
        if self.controller.is_zp_connected and self.job:
            try:
                self.controller.move_z_absolute(
                    self.job.settings.travel_z_height, from_zero_ref=True
                )
            except Exception as e:
                logger.error(f"Failed to raise Z after abort: {e}")

    # ── Internal ───────────────────────────────────────────────────

    def _set_state(self, new_state: PrintState):
        self.state = new_state
        if self.on_state_changed:
            self.on_state_changed(new_state)

    def _report_progress(self, message: str):
        if self.on_progress:
            self.on_progress(self._current_step, self.job.total_steps, message)

    def _execute_loop(self):
        """Main execution loop running in a dedicated thread."""
        logger.info(f"Starting print: {self.job.name}")

        # Session 4: Log print start
        pos_logger = getattr(self.controller, 'position_logger', None)
        if pos_logger:
            pos_logger.record(
                "print_start",
                xy_pos=self.controller.get_xy_position(cached=False),
                zp_pos=self.controller.get_zp_position(cached=False),
                metadata={"job_name": self.job.name, "total_steps": self.job.total_steps},
            )

        try:
            for i, cmd in enumerate(self.job.commands):
                # Check abort
                if self._abort_flag.is_set():
                    logger.info("Abort flag detected, stopping")
                    return

                # Wait if paused
                self._pause_event.wait()

                if self._abort_flag.is_set():
                    return

                self._current_step = i + 1
                label = cmd.label or cmd.type.value
                self._report_progress(f"[{i + 1}/{self.job.total_steps}] {label}")

                self._execute_command(cmd)

                # Session 4: Log position periodically (every 10 commands)
                if pos_logger and i % 10 == 0:
                    pos_logger.record(
                        "print_progress",
                        xy_pos=self.controller.get_xy_position(cached=True),
                        zp_pos=self.controller.get_zp_position(cached=True),
                        metadata={"step": i + 1, "command": cmd.type.value},
                    )

            self._set_state(PrintState.COMPLETED)
            self._report_progress("Print complete!")
            logger.info("Print job completed successfully")

            # Session 4: Log print end
            if pos_logger:
                pos_logger.record(
                    "print_end",
                    xy_pos=self.controller.get_xy_position(cached=False),
                    zp_pos=self.controller.get_zp_position(cached=False),
                    metadata={"job_name": self.job.name, "result": "completed"},
                )

        except Exception as e:
            logger.error(f"Print execution error: {e}", exc_info=True)
            self._set_state(PrintState.ERROR)
            self._report_progress(f"Error: {e}")

            if pos_logger:
                pos_logger.record(
                    "print_error",
                    xy_pos=self.controller.get_xy_position(cached=True),
                    zp_pos=self.controller.get_zp_position(cached=True),
                    metadata={"job_name": self.job.name, "error": str(e)},
                )

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
            ctrl.move_xy_absolute(x, y, from_zero_ref=True)
            self._wait_for_xy_settle(x, y, timeout=10.0)

            # Session 4: Log move
            pos_logger = getattr(ctrl, 'position_logger', None)
            if pos_logger:
                pos_logger.record("move_xy", xy_pos=ctrl.get_xy_position(cached=True),
                                  metadata={"target_x": x, "target_y": y})

        elif cmd.type == CommandType.MOVE_Z:
            z = p.get("z", 0)
            ctrl.move_z_absolute(z, from_zero_ref=True)
            time.sleep(0.5)

        elif cmd.type == CommandType.MOVE_Z_REL:
            dist = p.get("distance", 0)
            feedrate = p.get("feedrate", settings.z_feedrate)
            ctrl.move_z_relative(dist, feedrate)
            time.sleep(0.3)

        elif cmd.type == CommandType.EXTRUDE:
            pump = p.get("pump", self._active_pump)
            amount = p.get("amount", 0)
            feedrate = p.get("feedrate", settings.pump_feedrate)
            ctrl.move_pump_relative(pump, amount, feedrate)
            wait = abs(amount) / (feedrate / 60.0) + 0.1
            time.sleep(min(wait, 5.0))

        elif cmd.type == CommandType.PRINT_PATH:
            self._execute_print_path(cmd)

        elif cmd.type == CommandType.DWELL:
            seconds = p.get("seconds", 0)
            end_time = time.time() + seconds
            while time.time() < end_time:
                if self._abort_flag.is_set():
                    return
                time.sleep(min(0.1, seconds))

        elif cmd.type == CommandType.TRAVEL_UP:
            ctrl.move_z_absolute(settings.travel_z_height, from_zero_ref=True)
            time.sleep(0.5)

        elif cmd.type == CommandType.TRAVEL_DOWN:
            ctrl.move_z_absolute(settings.print_z_height, from_zero_ref=True)
            time.sleep(0.3)

        elif cmd.type == CommandType.HOME_XY:
            ctrl.move_xy_absolute(0, 0, from_zero_ref=True)
            self._wait_for_xy_settle(0, 0, timeout=15.0)

        elif cmd.type == CommandType.SET_PUMP_RATE:
            pass

        elif cmd.type == CommandType.SWITCH_PUMP:
            self._execute_switch_pump(cmd)

        else:
            logger.warning(f"Unknown command type: {cmd.type}")

    def _execute_switch_pump(self, cmd: PrintCommand):
        """
        Session 4: Execute pump switch for multi-material printing.
        
        Retracts the old pump, switches active pump, then primes the new pump.
        """
        new_pump = cmd.params.get("pump", "P1")
        old_pump = cmd.params.get("old_pump", self._active_pump)
        settings = self.job.settings
        ctrl = self.controller

        logger.info(f"Switching pump: {old_pump} → {new_pump}")

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

    def _execute_print_path(self, cmd: PrintCommand):
        """
        Execute a coordinated print path: move XY while extruding.
        """
        ctrl = self.controller
        settings = self.job.settings
        points = cmd.params.get("points", [])
        pump = cmd.params.get("pump", self._active_pump)
        flow_rate = cmd.params.get("flow_rate", 0.01)

        if len(points) < 2:
            return

        # Move to start of path
        start_x, start_y = points[0][0], points[0][1]
        ctrl.move_xy_absolute(start_x, start_y, from_zero_ref=True)
        self._wait_for_xy_settle(start_x, start_y, timeout=5.0)

        # Execute path segments
        for i in range(1, len(points)):
            if self._abort_flag.is_set():
                return

            x1, y1 = points[i - 1][0], points[i - 1][1]
            x2, y2 = points[i][0], points[i][1]
            seg_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if seg_length < 0.001:
                continue

            # Extrude proportional to segment length
            extrude_amount = seg_length * flow_rate
            if extrude_amount > 0.0001:
                mapped = AXIS_MAP.get(pump)
                if mapped and ctrl.zp_stage:
                    ctrl.zp_stage.move_relative(
                        {mapped: extrude_amount},
                        settings.pump_feedrate,
                    )

            # Move XY
            ctrl.move_xy_absolute(x2, y2, from_zero_ref=True)
            move_time = seg_length / max(settings.print_feedrate, 1) * 60
            time.sleep(max(move_time, 0.05))

    def _wait_for_xy_settle(self, target_x, target_y, timeout=10.0, tolerance=50):
        """
        Wait for XY stage to reach target position.
        Falls back to a fixed delay if position can't be read.
        """
        if not self.controller.is_xy_connected:
            time.sleep(0.5)
            return

        machine_x = target_x + self.controller.zero_position["x"]
        machine_y = target_y + self.controller.zero_position["y"]

        start = time.time()
        while time.time() - start < timeout:
            if self._abort_flag.is_set():
                return

            try:
                pos = self.controller.get_xy_position()
                if pos[0] is not None:
                    dx = abs(pos[0] - machine_x)
                    dy = abs(pos[1] - machine_y)
                    if dx < tolerance and dy < tolerance:
                        return
            except Exception:
                pass

            time.sleep(0.1)

        logger.warning(f"XY settle timeout after {timeout}s")


# ═══════════════════════════════════════════════════════════════════
# Print Queue (Session 4, Task 6)
# ═══════════════════════════════════════════════════════════════════

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
