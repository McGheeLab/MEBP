"""
ZP Stage Simulator — Physics-based simulator for the Marlin-based ZP stage.

Simulates G-code command processing, serial buffering, and smooth motion
interpolation for all four axes (X/Y/Z/E mapped to Z-needle and 3 pumps).

Presents the same buffered-serial interface that :class:`ZPStageManager`
expects (write / flush / read_all), so it can be used as a drop-in
replacement when ``simulate=True``.
"""

from __future__ import annotations

import logging
import queue
import re
import threading
import time

logger = logging.getLogger(__name__)


class ZPStageSimulator:
    """
    Threaded ZP stage simulator with G-code processing and smooth motion.

    Axes follow Marlin convention: X, Y, Z, E.
    The mapping to logical axes (Z-needle, P1, P2, P3) is handled by
    :class:`ZPStageManager`, not here.

    Parameters:
        communication_delay:          Simulated serial write latency (s).
        processing_time_per_command:  Simulated per-command processing time (s).
        acceleration_rate:            Max velocity change per second.
        max_speed:                    Velocity clamp per axis.
        kp:                           Proportional gain for position tracking.
    """

    # Axes tracked by this simulator
    AXES = ("X", "Y", "Z", "E")

    def __init__(
        self,
        communication_delay: float = 0.03,
        processing_time_per_command: float = 0.01,
        acceleration_rate: float = 100.0,
        max_speed: float = 100.0,
        kp: float = 2.0,
    ):
        # Serial-style buffer and queues
        self._buffer: bytes = b""
        self._command_queue: queue.Queue[str] = queue.Queue()
        self._response_queue: queue.Queue[str] = queue.Queue()

        # Position and velocity per axis
        self.position: dict[str, float] = {a: 0.0 for a in self.AXES}
        self.target_position: dict[str, float] = {a: 0.0 for a in self.AXES}
        self.current_velocity: dict[str, float] = {a: 0.0 for a in self.AXES}
        self.counts: dict[str, int] = {"X": 0, "Y": 0, "Z": 0}

        # Positioning mode
        self.absolute_mode: bool = True

        # Timing
        self.communication_delay: float = communication_delay
        self.processing_time_per_command: float = processing_time_per_command
        self.acceleration_rate: float = acceleration_rate
        self.max_speed: float = max_speed
        self.kp: float = kp

        # Threading
        self._lock = threading.Lock()
        self._running = False
        self._cmd_thread: threading.Thread | None = None
        self._physics_thread: threading.Thread | None = None

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

    # (flush is defined above, _flush_buffer is the internal implementation)

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
            M500  — save settings
            M503  — report settings
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
                return "FIRMWARE_NAME:Marlin Simulator"
            elif cmd.startswith("M302"):
                return "ok"
            elif cmd == "M83":
                return "ok"
            elif cmd.startswith("M92"):
                return "ok"
            elif cmd.startswith("M203"):
                return "ok"
            elif cmd.startswith("M220"):
                return "ok"
            elif cmd == "M500":
                return "ok"
            elif cmd == "M503":
                return "ok"
            elif cmd == "M112":
                # Emergency stop: zero all velocities
                for a in self.AXES:
                    self.current_velocity[a] = 0.0
                    self.target_position[a] = self.position[a]
                return "ok"
            else:
                return "ok"  # Unknown commands get 'ok' like real Marlin

    def _cmd_move(self, cmd: str) -> str:
        """Handle G0 movement command (lock must be held)."""
        axes = re.findall(r"([XYZE])([-+]?\d*\.?\d+)", cmd)
        if not axes:
            return "ok"

        for axis, value_str in axes:
            try:
                val = float(value_str)
            except ValueError:
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
                    error = self.target_position[axis] - self.position[axis]
                    desired = max(-self.max_speed, min(self.max_speed, self.kp * error))
                    self.current_velocity[axis] = self._ramp_velocity(
                        self.current_velocity[axis], desired, dt
                    )
                    self.position[axis] += self.current_velocity[axis] * dt

            elapsed = time.time() - start
            time.sleep(max(0.0, 0.01 - elapsed))  # ~100 Hz

    # ── Debug ─────────────────────────────────────────────────────

    def get_current_position_dict(self) -> dict[str, float]:
        """Return current positions as a dict (for debugging)."""
        with self._lock:
            return dict(self.position)
