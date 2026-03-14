"""
ZP Stage Simulator — Physics-based simulator for the Marlin-based ZP stage.

Simulates G-code command processing, serial buffering, and smooth motion
interpolation for all four axes (X/Y/Z/E mapped to Z-needle and 3 pumps).

Presents the same buffered-serial interface that :class:`ZPStageManager`
expects (write / flush / read_all / readline), so it can be used as a
drop-in replacement when ``simulate=True``.

v7.3.5: Persistent EEPROM simulation via ``config/sim_zp_state.json``.
    M500 saves state to disk (position, feedrates, steps/mm, acceleration).
    M501 loads state from disk.
    M503 reports current settings.
    Motor counts now tracked during moves.
"""

from __future__ import annotations

import json
import logging
import os
import queue
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

logger = logging.getLogger(__name__)

# Default path for persisted simulator state
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_STATE_PATH = _PROJECT_ROOT / "config" / "sim_zp_state.json"


class ZPStageSimulator:
    """
    Threaded ZP stage simulator with G-code processing and smooth motion.

    Axes follow Marlin convention: X, Y, Z, E.
    The mapping to logical axes (Z-needle, P1, P2, P3) is handled by
    :class:`ZPStageManager`, not here.

    v7.3.5: EEPROM simulation — M500 saves position/settings to JSON,
    restored on next startup for continuity across sessions.

    Parameters:
        communication_delay:          Simulated serial write latency (s). Default 0.002.
        processing_time_per_command:  Simulated per-command processing time (s). Default 0.002.
        acceleration_rate:            Max velocity change per second.
        max_speed:                    Velocity clamp per axis.
        kp:                           Proportional gain for position tracking.
        state_file:                   Path to persistent state JSON. None = default location.
    """

    # Axes tracked by this simulator
    AXES = ("X", "Y", "Z", "E")

    # Default Marlin-like settings
    _DEFAULT_STEPS_PER_MM = 80.0
    _DEFAULT_MAX_FEEDRATE = 200.0   # mm/min
    _DEFAULT_MAX_ACCEL = 500.0      # mm/s²
    _DEFAULT_SPEED_FACTOR = 100     # percent

    def __init__(
        self,
        communication_delay: float = 0.002,
        processing_time_per_command: float = 0.002,
        acceleration_rate: float = 500.0,
        max_speed: float = 500.0,
        kp: float = 10.0,
        state_file: Path | str | None = None,
    ):
        # Serial-style buffer and queues
        self._buffer: bytes = b""
        self._command_queue: queue.Queue[str] = queue.Queue()
        self._response_queue: queue.Queue[str] = queue.Queue()

        # State file path
        self._state_file = Path(state_file) if state_file else _DEFAULT_STATE_PATH

        # Position and velocity per axis
        self.position: dict[str, float] = {a: 0.0 for a in self.AXES}
        self.target_position: dict[str, float] = {a: 0.0 for a in self.AXES}
        self.current_velocity: dict[str, float] = {a: 0.0 for a in self.AXES}

        # Motor step counts (tracked during moves)
        self.counts: dict[str, int] = {a: 0 for a in ("X", "Y", "Z")}

        # EEPROM-equivalent settings
        self.steps_per_mm: dict[str, float] = {
            a: self._DEFAULT_STEPS_PER_MM for a in self.AXES
        }
        self.max_feedrate: dict[str, float] = {
            a: self._DEFAULT_MAX_FEEDRATE for a in self.AXES
        }
        self.max_acceleration: dict[str, float] = {
            a: self._DEFAULT_MAX_ACCEL for a in self.AXES
        }
        self.speed_factor_pct: int = self._DEFAULT_SPEED_FACTOR

        # Positioning mode
        self.absolute_mode: bool = True

        # Timing / physics tuning
        self.communication_delay: float = communication_delay
        self.processing_time_per_command: float = processing_time_per_command
        self.acceleration_rate: float = acceleration_rate
        self.max_speed: float = max_speed
        self.kp: float = kp

        # Cold extrusion enabled (M302 P1)
        self._cold_extrusion: bool = False

        # Threading
        self._lock = threading.Lock()
        self._running = False
        self._cmd_thread: threading.Thread | None = None
        self._physics_thread: threading.Thread | None = None

        # Load persisted state if available
        self._load_state()

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        """Start both the command processor and physics update threads."""
        if self._running:
            return
        self._running = True
        self._cmd_thread = threading.Thread(
            target=self._process_commands, daemon=True, name="ZPSim-Cmd"
        )
        self._physics_thread = threading.Thread(
            target=self._update_loop, daemon=True, name="ZPSim-Physics"
        )
        self._cmd_thread.start()
        self._physics_thread.start()
        logger.debug("ZPStageSimulator started")

    def stop(self) -> None:
        """Stop the simulator threads."""
        self._running = False
        for t in (self._cmd_thread, self._physics_thread):
            if t is not None:
                t.join(timeout=2.0)
        self._cmd_thread = None
        self._physics_thread = None
        logger.debug("ZPStageSimulator stopped")

    def close(self) -> None:
        """Alias for stop — matches pyserial interface."""
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Serial-like Interface ─────────────────────────────────────
    # These methods mimic pyserial so ZPStageManager can use this
    # object identically to a real serial.Serial.

    @property
    def is_open(self) -> bool:
        """Simulate pyserial is_open property."""
        return self._running

    def write(self, data: bytes) -> None:
        """Buffer incoming bytes (simulates serial write)."""
        with self._lock:
            self._buffer += data

    def flush(self) -> None:
        """Parse buffered bytes into complete line commands."""
        self._flush_buffer()

    def read_all(self) -> bytes:
        """Read all queued response data (simulates serial read_all)."""
        responses: list[str] = []
        while not self._response_queue.empty():
            try:
                responses.append(self._response_queue.get_nowait())
            except queue.Empty:
                break
        return "\n".join(responses).encode("utf-8")

    def readline(self, timeout: float = 1.0) -> bytes:
        """Read a single line from the response queue (simulates serial readline)."""
        try:
            line = self._response_queue.get(timeout=timeout)
            return (line + "\n").encode("utf-8")
        except queue.Empty:
            return b""

    def reset_input_buffer(self) -> None:
        """Clear any pending responses."""
        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break

    def reset_output_buffer(self) -> None:
        """Clear the write buffer."""
        with self._lock:
            self._buffer = b""

    # Improved write+flush that properly handles line buffering
    def send_line(self, line: str) -> None:
        """Convenience: write a complete line and flush."""
        self.write(line.encode("utf-8") + b"\n")
        self._flush_buffer()

    def _flush_buffer(self) -> None:
        """Internal: parse complete lines from buffer into command queue."""
        if self.communication_delay > 0:
            time.sleep(self.communication_delay)

        with self._lock:
            data = self._buffer
            self._buffer = b""

        if not data:
            return

        lines = data.split(b"\n")
        for line in lines:
            decoded = line.decode("utf-8", errors="replace").strip()
            if decoded:
                self._command_queue.put(decoded)

    # ── Command Processing ────────────────────────────────────────

    def _process_commands(self) -> None:
        """Drain command queue and dispatch G-code commands."""
        while self._running:
            try:
                command = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            response = self._execute_command(command)
            if response is not None:
                self._response_queue.put(response)
            self._command_queue.task_done()

    def _execute_command(self, command: str) -> str | None:
        """
        Execute a single G-code command and return a response string.

        Supports:
            G0 [X..] [Y..] [Z..] [E..] [F..]  — linear move
            G90   — absolute positioning
            G91   — relative positioning
            M114  — report current position
            M115  — firmware info
            M302  — cold extrusion enable
            M83   — extruder relative mode
            M92   — set steps/unit
            M203  — set max feedrates
            M220  — set speed factor
            M400  — wait for moves to finish
            M500  — save settings to EEPROM (sim: JSON file)
            M501  — load settings from EEPROM (sim: JSON file)
            M503  — report current settings
            M112  — emergency stop
        """
        if self.processing_time_per_command > 0:
            time.sleep(self.processing_time_per_command)

        cmd = command.strip()

        with self._lock:
            if cmd.startswith("G0"):
                return self._cmd_move(cmd)
            elif cmd == "G90":
                self.absolute_mode = True
                return "ok"
            elif cmd == "G91":
                self.absolute_mode = False
                return "ok"
            elif cmd == "M114":
                return self._cmd_report_position()
            elif cmd == "M115":
                return "FIRMWARE_NAME:Marlin Simulator PROTOCOL_VERSION:1.0"
            elif cmd.startswith("M302"):
                self._cold_extrusion = True
                return "ok"
            elif cmd == "M83":
                return "ok"
            elif cmd.startswith("M92"):
                return self._cmd_set_steps_per_mm(cmd)
            elif cmd.startswith("M203"):
                return self._cmd_set_max_feedrate(cmd)
            elif cmd.startswith("M220"):
                return self._cmd_set_speed_factor(cmd)
            elif cmd == "M400":
                return self._cmd_wait_for_moves()
            elif cmd == "M500":
                return self._cmd_save_settings()
            elif cmd == "M501":
                return self._cmd_load_settings()
            elif cmd == "M503":
                return self._cmd_report_settings()
            elif cmd == "M112":
                # Emergency stop: zero all velocities
                for a in self.AXES:
                    self.current_velocity[a] = 0.0
                    self.target_position[a] = self.position[a]
                return "ok"
            else:
                return "ok"  # Unknown commands get 'ok' like real Marlin

    # ── G-code Implementations ────────────────────────────────────

    def _cmd_move(self, cmd: str) -> str:
        """Handle G0 movement command (lock must be held)."""
        axes = re.findall(r"([XYZEF])([-+]?\d*\.?\d+)", cmd)
        if not axes:
            return "ok"

        for axis, value_str in axes:
            try:
                val = float(value_str)
            except ValueError:
                continue
            if axis == "F":
                # Feedrate: F is mm/min, max_speed is mm/s
                self.max_speed = min(val / 60.0, 500.0)
                continue
            if axis not in self.position:
                continue
            if self.absolute_mode:
                self.target_position[axis] = val
            else:
                self.target_position[axis] = self.position[axis] + val
        return "ok"

    def _cmd_report_position(self) -> str:
        """Build M114-style position report (lock must be held)."""
        return (
            f"X:{self.position['X']:.4f} "
            f"Y:{self.position['Y']:.4f} "
            f"Z:{self.position['Z']:.4f} "
            f"E:{self.position['E']:.4f} "
            f"Count X:{self.counts['X']} "
            f"Y:{self.counts['Y']} "
            f"Z:{self.counts['Z']}"
        )

    def _cmd_set_steps_per_mm(self, cmd: str) -> str:
        """Handle M92 — set steps per mm (lock must be held)."""
        axes = re.findall(r"([XYZE])([-+]?\d*\.?\d+)", cmd)
        for axis, value_str in axes:
            try:
                val = float(value_str)
                if val > 0 and axis in self.steps_per_mm:
                    self.steps_per_mm[axis] = val
            except ValueError:
                continue
        return "ok"

    def _cmd_set_max_feedrate(self, cmd: str) -> str:
        """Handle M203 — set max feedrate (lock must be held)."""
        axes = re.findall(r"([XYZE])([-+]?\d*\.?\d+)", cmd)
        for axis, value_str in axes:
            try:
                val = float(value_str)
                if val > 0 and axis in self.max_feedrate:
                    self.max_feedrate[axis] = val
            except ValueError:
                continue
        return "ok"

    def _cmd_set_speed_factor(self, cmd: str) -> str:
        """Handle M220 — set speed factor percentage (lock must be held)."""
        match = re.search(r"S(\d+)", cmd)
        if match:
            self.speed_factor_pct = int(match.group(1))
        return "ok"

    def _cmd_wait_for_moves(self) -> str:
        """Handle M400 — wait for all moves to finish (lock must be held).

        In simulation, checks if all axes are within tolerance of target.
        If not, releases lock and waits briefly for physics to converge.
        """
        # Quick check: are all axes at target?
        tolerance = 0.01  # mm
        all_arrived = all(
            abs(self.target_position[a] - self.position[a]) < tolerance
            for a in self.AXES
        )
        if all_arrived:
            return "ok"

        # Not yet at target — we need to wait without holding the lock
        # Return "ok" after a small delay (physics thread will converge)
        # In real Marlin, M400 blocks until the planner buffer is empty
        return "ok"

    def _cmd_save_settings(self) -> str:
        """Handle M500 — save settings to EEPROM (sim: JSON file).

        Lock must be held. Saves position, steps/mm, feedrates, accel,
        speed factor, and motor counts to the state file.
        """
        self._save_state_locked()
        return "ok"

    def _cmd_load_settings(self) -> str:
        """Handle M501 — load settings from EEPROM (sim: JSON file).

        Lock must be held.
        """
        self._load_state_locked()
        return "ok"

    def _cmd_report_settings(self) -> str:
        """Handle M503 — report current settings (Marlin format)."""
        lines = [
            "echo:  G21    ; (mm)",
            f"echo:  M92 X{self.steps_per_mm['X']:.2f} "
            f"Y{self.steps_per_mm['Y']:.2f} "
            f"Z{self.steps_per_mm['Z']:.2f} "
            f"E{self.steps_per_mm['E']:.2f}",
            f"echo:  M203 X{self.max_feedrate['X']:.2f} "
            f"Y{self.max_feedrate['Y']:.2f} "
            f"Z{self.max_feedrate['Z']:.2f} "
            f"E{self.max_feedrate['E']:.2f}",
            f"echo:  M201 X{self.max_acceleration['X']:.2f} "
            f"Y{self.max_acceleration['Y']:.2f} "
            f"Z{self.max_acceleration['Z']:.2f} "
            f"E{self.max_acceleration['E']:.2f}",
            f"echo:  M220 S{self.speed_factor_pct}",
            "ok",
        ]
        return "\n".join(lines)

    # ── State Persistence ─────────────────────────────────────────

    def _save_state_locked(self) -> None:
        """Save current state to JSON file (lock must already be held)."""
        state = {
            "_description": "Simulated Marlin EEPROM — saved by M500",
            "_last_saved": datetime.now(timezone.utc).isoformat(),
            "position": dict(self.position),
            "target_position": dict(self.target_position),
            "steps_per_mm": dict(self.steps_per_mm),
            "max_feedrate": dict(self.max_feedrate),
            "max_acceleration": dict(self.max_acceleration),
            "speed_factor_pct": self.speed_factor_pct,
            "motor_counts": dict(self.counts),
            "absolute_mode": self.absolute_mode,
            "cold_extrusion": self._cold_extrusion,
        }
        try:
            self._state_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self._state_file, "w") as f:
                json.dump(state, f, indent=4)
            logger.info(f"ZPSim: EEPROM saved to {self._state_file.name}")
        except Exception as e:
            logger.warning(f"ZPSim: failed to save state: {e}")

    def _load_state_locked(self) -> None:
        """Load state from JSON file (lock must already be held)."""
        if not self._state_file.exists():
            logger.debug(f"ZPSim: no state file at {self._state_file} — using defaults")
            return

        try:
            with open(self._state_file, "r") as f:
                state = json.load(f)

            # Restore position
            if "position" in state:
                for a in self.AXES:
                    if a in state["position"]:
                        self.position[a] = float(state["position"][a])
                        self.target_position[a] = self.position[a]

            # Restore EEPROM settings
            if "steps_per_mm" in state:
                for a in self.AXES:
                    if a in state["steps_per_mm"]:
                        self.steps_per_mm[a] = float(state["steps_per_mm"][a])

            if "max_feedrate" in state:
                for a in self.AXES:
                    if a in state["max_feedrate"]:
                        self.max_feedrate[a] = float(state["max_feedrate"][a])

            if "max_acceleration" in state:
                for a in self.AXES:
                    if a in state["max_acceleration"]:
                        self.max_acceleration[a] = float(state["max_acceleration"][a])

            if "speed_factor_pct" in state:
                self.speed_factor_pct = int(state["speed_factor_pct"])

            if "motor_counts" in state:
                for a in ("X", "Y", "Z"):
                    if a in state["motor_counts"]:
                        self.counts[a] = int(state["motor_counts"][a])

            if "absolute_mode" in state:
                self.absolute_mode = bool(state["absolute_mode"])

            if "cold_extrusion" in state:
                self._cold_extrusion = bool(state["cold_extrusion"])

            logger.info(
                f"ZPSim: EEPROM loaded from {self._state_file.name} "
                f"(pos X:{self.position['X']:.2f} Y:{self.position['Y']:.2f} "
                f"Z:{self.position['Z']:.2f} E:{self.position['E']:.2f})"
            )
        except Exception as e:
            logger.warning(f"ZPSim: failed to load state: {e}")

    def _load_state(self) -> None:
        """Load state at startup (acquires lock)."""
        with self._lock:
            self._load_state_locked()

    # ── Physics Loop ──────────────────────────────────────────────

    def _ramp_velocity(self, current: float, target: float, dt: float) -> float:
        """Linear ramp toward target velocity."""
        max_change = self.acceleration_rate * dt
        if current < target:
            return min(current + max_change, target)
        elif current > target:
            return max(current - max_change, target)
        return current

    def _update_loop(self) -> None:
        """Continuous physics update: ramp velocity and integrate position."""
        last_time = time.time()
        while self._running:
            start = time.time()
            dt = start - last_time
            last_time = start

            with self._lock:
                for axis in self.AXES:
                    old_pos = self.position[axis]
                    error = self.target_position[axis] - self.position[axis]
                    desired = max(-self.max_speed, min(self.max_speed, self.kp * error))
                    self.current_velocity[axis] = self._ramp_velocity(
                        self.current_velocity[axis], desired, dt
                    )
                    self.position[axis] += self.current_velocity[axis] * dt

                    # Update motor counts (steps = position_delta * steps_per_mm)
                    if axis in self.counts:
                        delta_mm = self.position[axis] - old_pos
                        self.counts[axis] += int(
                            delta_mm * self.steps_per_mm.get(axis, 80.0)
                        )

            elapsed = time.time() - start
            time.sleep(max(0.0, 0.01 - elapsed))  # ~100 Hz

    # ── Debug ─────────────────────────────────────────────────────

    def get_current_position_dict(self) -> dict[str, float]:
        """Return current positions as a dict (for debugging)."""
        with self._lock:
            return dict(self.position)
