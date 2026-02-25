"""
Print Manager - Print file loading, path execution, and job control.

Handles:
- Loading print files (custom JSON format or simplified G-code)
- Generating print paths from well plate + pattern combinations
- Sequential execution with pause / resume / abort
- Progress reporting via callback

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
        "num_layers": 1
    },
    "commands": [
        {"type": "move_xy", "x": 10.0, "y": 20.0},
        {"type": "move_z", "z": 0.1},
        {"type": "extrude", "pump": "P1", "amount": 0.5, "feedrate": 30},
        {"type": "print_path", "points": [[0,0],[10,0],[10,10]], "pump": "P1", "flow_rate": 0.01},
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
    retract_amount: float = 0.0      # Pump retraction after path segment
    prime_amount: float = 0.0        # Pump prime before path segment
    dwell_after_move: float = 0.0    # Seconds to wait after travel moves

    @classmethod
    def from_dict(cls, d: dict) -> "PrintSettings":
        """Create from dictionary, ignoring unknown keys."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in d.items() if k in valid_keys}
        return cls(**filtered)


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
        Returns list of {"type": "travel"|"print", "points": [(x,y), ...]}
        """
        segments = []
        current_segment = {"type": "travel", "points": []}

        for cmd in self.commands:
            if cmd.type == CommandType.MOVE_XY:
                pt = (cmd.params.get("x", 0), cmd.params.get("y", 0))
                if current_segment["points"]:
                    segments.append(current_segment)
                current_segment = {"type": "travel", "points": [pt]}

            elif cmd.type == CommandType.PRINT_PATH:
                if current_segment["points"]:
                    segments.append(current_segment)
                path = cmd.params.get("points", [])
                current_segment = {
                    "type": "print",
                    "points": [(p[0], p[1]) for p in path],
                }

            elif cmd.type == CommandType.HOME_XY:
                if current_segment["points"]:
                    segments.append(current_segment)
                current_segment = {"type": "travel", "points": [(0, 0)]}

        if current_segment["points"]:
            segments.append(current_segment)

        return segments


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
                # Preserve comments as labels
                if line.startswith(";"):
                    commands.append(PrintCommand(
                        type=CommandType.COMMENT,
                        label=line[1:].strip(),
                    ))
                continue

            # Strip inline comments
            if ";" in line:
                line = line[:line.index(";")].strip()

            parts = line.split()
            if not parts:
                continue

            gcode = parts[0].upper()

            if gcode in ("G0", "G1"):
                # Parse axis values
                params = _parse_gcode_params(parts[1:])
                has_xy = "X" in params or "Y" in params
                has_z = "Z" in params
                has_e = "E" in params

                new_x = params.get("X", current_x)
                new_y = params.get("Y", current_y)
                new_z = params.get("Z", current_z)
                new_e = params.get("E", current_e)

                if has_z and not has_xy:
                    # Pure Z move
                    commands.append(PrintCommand(
                        type=CommandType.MOVE_Z,
                        params={"z": new_z},
                        label=f"Z → {new_z:.3f}",
                    ))

                if has_xy and has_e and (new_e - current_e) > 0:
                    # Print move (XY + extrusion)
                    commands.append(PrintCommand(
                        type=CommandType.PRINT_PATH,
                        params={
                            "points": [[current_x, current_y], [new_x, new_y]],
                            "pump": "P1",
                            "total_extrusion": new_e - current_e,
                        },
                        label=f"Print → ({new_x:.1f}, {new_y:.1f})",
                    ))
                elif has_xy:
                    # Travel move
                    commands.append(PrintCommand(
                        type=CommandType.MOVE_XY,
                        params={"x": new_x, "y": new_y},
                        label=f"Travel → ({new_x:.1f}, {new_y:.1f})",
                    ))

                current_x, current_y, current_z, current_e = new_x, new_y, new_z, new_e

            elif gcode == "G4":
                # Dwell
                params = _parse_gcode_params(parts[1:])
                seconds = params.get("S", params.get("P", 0) / 1000.0)
                commands.append(PrintCommand(
                    type=CommandType.DWELL,
                    params={"seconds": seconds},
                    label=f"Dwell {seconds:.1f}s",
                ))

            elif gcode == "G28":
                commands.append(PrintCommand(
                    type=CommandType.HOME_XY,
                    label="Home",
                ))
                current_x, current_y = 0.0, 0.0

    return PrintJob(
        name=path.stem,
        description=f"Imported from {path.name}",
        commands=commands,
        source_file=str(path),
    )


def _parse_gcode_params(parts: list[str]) -> dict:
    """Parse G-code parameters like 'X10.5 Y20 F1000' into dict."""
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
) -> PrintJob:
    """
    Build a print job that prints a pattern in each well of a well plate.
    
    Args:
        well_positions: List of (well_name, x, y) for each well to print
        path_points: Pattern points relative to well center (0,0)
        settings: Print settings
        pump: Which pump to use (P1, P2, P3)
        flow_rate: Extrusion rate per mm of path travel
        job_name: Name for the job
        
    Returns:
        PrintJob ready for execution
    """
    commands = []

    for layer in range(settings.num_layers):
        z_height = settings.print_z_height + layer * settings.layer_height

        commands.append(PrintCommand(
            type=CommandType.COMMENT,
            label=f"=== Layer {layer + 1}/{settings.num_layers} (z={z_height:.3f}) ===",
        ))

        for well_name, well_x, well_y in well_positions:
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
                label=f"Lower to print height",
            ))

            # Prime
            if settings.prime_amount > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": pump, "amount": settings.prime_amount,
                            "feedrate": settings.pump_feedrate},
                    label="Prime",
                ))

            # Print the path in this well
            well_path = [(well_x + px, well_y + py) for px, py in path_points]
            commands.append(PrintCommand(
                type=CommandType.PRINT_PATH,
                params={
                    "points": well_path,
                    "pump": pump,
                    "flow_rate": flow_rate,
                },
                label=f"Print in {well_name}",
            ))

            # Retract
            if settings.retract_amount > 0:
                commands.append(PrintCommand(
                    type=CommandType.EXTRUDE,
                    params={"pump": pump, "amount": -settings.retract_amount,
                            "feedrate": settings.pump_feedrate},
                    label="Retract",
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
    
    Usage:
        manager = PrintManager(stage_controller)
        manager.load_file("job.json")
        manager.on_progress = lambda step, total, msg: print(f"{step}/{total}: {msg}")
        manager.on_state_changed = lambda state: print(f"State: {state}")
        manager.start()
    """

    def __init__(self, controller):
        """
        Args:
            controller: StageController instance with connected stages
        """
        self.controller = controller
        self.job: Optional[PrintJob] = None
        self.state = PrintState.IDLE

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
        self._set_state(PrintState.IDLE)
        logger.info(f"Loaded print job: {self.job.name} ({self.job.total_steps} commands)")

    def load_job(self, job: PrintJob):
        """Load a pre-built PrintJob directly."""
        if self.state == PrintState.RUNNING:
            raise RuntimeError("Cannot load job while printing")
        self.job = job
        self._current_step = 0
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
        self._abort_flag.set()
        self._pause_event.set()  # Unblock if paused
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        self._set_state(PrintState.ABORTED)
        logger.info("Print aborted")

        # Safety: raise Z to travel height
        if self.job and self.controller.is_zp_connected:
            self.controller.move_z_absolute(
                self.job.settings.travel_z_height, from_zero_ref=True
            )

    @property
    def progress(self) -> tuple[int, int]:
        """Current progress as (current_step, total_steps)."""
        total = self.job.total_steps if self.job else 0
        return (self._current_step, total)

    @property
    def progress_percent(self) -> float:
        """Progress as percentage 0-100."""
        if not self.job or self.job.total_steps == 0:
            return 0.0
        return (self._current_step / self.job.total_steps) * 100.0

    # ── Internal Execution ─────────────────────────────────────────

    def _set_state(self, new_state: PrintState):
        self.state = new_state
        if self.on_state_changed:
            try:
                self.on_state_changed(new_state)
            except Exception as e:
                logger.error(f"State callback error: {e}")

    def _report_progress(self, message: str):
        if self.on_progress:
            try:
                self.on_progress(self._current_step, self.job.total_steps, message)
            except Exception as e:
                logger.error(f"Progress callback error: {e}")

    def _execute_loop(self):
        """Main execution loop running in dedicated thread."""
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

            self._set_state(PrintState.COMPLETED)
            self._report_progress("Print complete!")
            logger.info("Print job completed successfully")

        except Exception as e:
            logger.error(f"Print execution error: {e}", exc_info=True)
            self._set_state(PrintState.ERROR)
            self._report_progress(f"Error: {e}")

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
            # Wait for move to (approximately) complete
            self._wait_for_xy_settle(x, y, timeout=10.0)

        elif cmd.type == CommandType.MOVE_Z:
            z = p.get("z", 0)
            ctrl.move_z_absolute(z, from_zero_ref=True)
            time.sleep(0.5)  # Allow time for Z move

        elif cmd.type == CommandType.MOVE_Z_REL:
            dist = p.get("distance", 0)
            feedrate = p.get("feedrate", settings.z_feedrate)
            ctrl.move_z_relative(dist, feedrate)
            time.sleep(0.3)

        elif cmd.type == CommandType.EXTRUDE:
            pump = p.get("pump", "P1")
            amount = p.get("amount", 0)
            feedrate = p.get("feedrate", settings.pump_feedrate)
            ctrl.move_pump_relative(pump, amount, feedrate)
            # Wait proportional to extrusion amount
            wait = abs(amount) / (feedrate / 60.0) + 0.1
            time.sleep(min(wait, 5.0))

        elif cmd.type == CommandType.PRINT_PATH:
            self._execute_print_path(cmd)

        elif cmd.type == CommandType.DWELL:
            seconds = p.get("seconds", 0)
            # Dwell with abort check
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
            # Store for subsequent commands
            pass

        else:
            logger.warning(f"Unknown command type: {cmd.type}")

    def _execute_print_path(self, cmd: PrintCommand):
        """
        Execute a coordinated print path: move XY while extruding.
        
        For the ZP stage (Marlin-based), extrusion is done by sending pump commands
        interleaved with XY moves. The XY stage uses velocity commands, so we break
        the path into segments and synchronize.
        """
        ctrl = self.controller
        settings = self.job.settings
        p = cmd.params

        points = p.get("points", [])
        if len(points) < 2:
            return

        pump = p.get("pump", "P1")
        flow_rate = p.get("flow_rate", 0.01)  # mm extrusion per mm travel
        total_extrusion = p.get("total_extrusion", None)

        # Calculate total path length
        total_length = 0
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            total_length += math.sqrt(dx * dx + dy * dy)

        if total_length == 0:
            return

        # If total_extrusion given, calculate flow_rate from it
        if total_extrusion is not None:
            flow_rate = total_extrusion / total_length

        # Execute segment by segment
        for i in range(1, len(points)):
            if self._abort_flag.is_set():
                return

            self._pause_event.wait()
            if self._abort_flag.is_set():
                return

            x0, y0 = points[i - 1]
            x1, y1 = points[i]
            dx = x1 - x0
            dy = y1 - y0
            seg_length = math.sqrt(dx * dx + dy * dy)

            if seg_length < 0.001:
                continue

            # Extrusion for this segment
            extrude_amount = flow_rate * seg_length

            # Move XY to next point
            ctrl.move_xy_absolute(x1, y1, from_zero_ref=True)

            # Simultaneously extrude pump
            if extrude_amount > 0:
                ctrl.move_pump_relative(pump, extrude_amount, settings.pump_feedrate)

            # Wait for XY move to approximately complete
            # Estimate time from distance and feedrate
            move_time = seg_length / max(settings.print_feedrate, 1.0)
            time.sleep(max(move_time, 0.05))

    def _wait_for_xy_settle(self, target_x: float, target_y: float,
                            timeout: float = 10.0, tolerance: float = 50.0):
        """
        Wait for XY stage to reach target position within tolerance.
        Falls back to a fixed delay if position can't be read.
        """
        if not self.controller.is_xy_connected:
            time.sleep(0.5)
            return

        # Convert target to machine coordinates (add zero ref)
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
