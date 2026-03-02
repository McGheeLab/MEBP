"""
XY Stage Simulator — Physics-based simulator for the Prior ProScan III XY stage.

Simulates velocity-mode and absolute-move commands with realistic acceleration
ramps and proportional-control positioning.  Presents the same serial-like
interface that :class:`XYStageManager` expects, so it can be used as a drop-in
replacement when ``simulate=True``.

The simulator runs a background update loop at a configurable rate (default
100 Hz) to smoothly interpolate position.

v7.1.2 BUG-2 FIX: Speed parameters are now scaled to microstep units.
    Prior ProScan III default: 10 microsteps/µm, max speed ~50,000 µsteps/s.
    The old default max_speed=100 was far too slow for microstep-scale positions,
    causing 500-step moves (50 µm jog) to take ~5 seconds instead of <0.1s.
"""

from __future__ import annotations

import logging
import threading
import time

logger = logging.getLogger(__name__)

# ── Realistic defaults for Prior ProScan III ──────────────────────
# ProScan III: 10 microsteps/µm, max velocity ~5mm/s = 50,000 µsteps/s
# These can be overridden by the caller (e.g., from protocol JSON params)
DEFAULT_MAX_SPEED = 100_000.0         # microsteps/s  (was 100!)
DEFAULT_ACCELERATION = 200_000.0      # microsteps/s²  (was 100!)
DEFAULT_KP = 20.0                     # Proportional gain for abs moves (was 2.0)
DEFAULT_SETTLING_THRESHOLD = 2.0      # Consider "arrived" within 2 microsteps


class XYStageSimulator:
    """
    Threaded XY stage simulator with velocity and absolute positioning modes.

    Public interface mirrors the subset of ``serial.Serial`` that
    :class:`XYStageManager` uses, plus a direct ``send_command`` path
    used when ``simulate=True``.

    Parameters:
        update_rate_hz:       Physics update frequency (Hz).
        acceleration_rate:    Maximum velocity change per second (microsteps/s²).
        communication_delay:  Simulated serial latency (seconds).
        max_speed:            Velocity clamp (microsteps/s).
    """

    def __init__(
        self,
        update_rate_hz: int = 100,
        acceleration_rate: float = DEFAULT_ACCELERATION,
        communication_delay: float = 0.0,
        max_speed: float = DEFAULT_MAX_SPEED,
    ):
        # Position state (microsteps)
        self.current_x: float = 0.0
        self.current_y: float = 0.0

        # Velocity state (microsteps/s)
        self.current_vx: float = 0.0
        self.current_vy: float = 0.0

        # Target state (depends on mode)
        self.target_vx: float = 0.0
        self.target_vy: float = 0.0
        self.target_x: float = 0.0
        self.target_y: float = 0.0

        # Mode: "velocity" or "absolute"
        self.mode: str = "velocity"

        # Proportional gain for absolute-move mode
        # Higher kp = faster approach but potential overshoot
        self.kp: float = DEFAULT_KP

        # Settling threshold — position is "reached" when within this
        self.settling_threshold: float = DEFAULT_SETTLING_THRESHOLD

        # Timing
        self._last_update_time: float = time.time()
        self.acceleration_rate: float = acceleration_rate
        self.update_rate_hz: int = update_rate_hz
        self._update_interval: float = 1.0 / update_rate_hz
        self.communication_delay: float = communication_delay
        self.max_speed: float = max_speed

        # Threading
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ── Lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        """Start the simulator physics loop."""
        if self._running:
            return
        self._running = True
        self._last_update_time = time.time()
        self._thread = threading.Thread(
            target=self._update_loop, daemon=True, name="XYSimulator"
        )
        self._thread.start()
        logger.debug(
            f"XYStageSimulator started (max_speed={self.max_speed:.0f}, "
            f"accel={self.acceleration_rate:.0f}, kp={self.kp:.1f})"
        )

    def stop(self) -> None:
        """Stop the simulator."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.debug("XYStageSimulator stopped")

    def close(self) -> None:
        """Alias for stop — matches pyserial interface."""
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Configuration ─────────────────────────────────────────────

    def configure_from_protocol(
        self,
        max_speed: float | None = None,
        acceleration: float | None = None,
        kp: float | None = None,
    ) -> None:
        """
        Update simulator parameters from controller protocol.

        Called by XYStageManager when a protocol is loaded, even in
        simulation mode (BUG-3 fix).
        """
        with self._lock:
            if max_speed is not None and max_speed > 0:
                self.max_speed = max_speed
            if acceleration is not None and acceleration > 0:
                self.acceleration_rate = acceleration
            if kp is not None and kp > 0:
                self.kp = kp
        logger.debug(
            f"Simulator configured: max_speed={self.max_speed:.0f}, "
            f"accel={self.acceleration_rate:.0f}, kp={self.kp:.1f}"
        )

    # ── Command Interface ─────────────────────────────────────────

    def send_command(self, command: str) -> str:
        """
        Process a ProScan-style command and return the response string.

        Supported commands:
            VS,vx,vy   — set velocity mode with target velocities
            PA,x,y     — absolute move (used by internal code)
            G x,y      — absolute move (ProScan syntax)
            GR dx,dy   — relative move
            P           — query current position
            V           — query firmware version (simulated)
            Z           — set home position
            SMS,speed   — set max speed
            SAS,accel   — set acceleration
            SCS,jerk    — set jerk (no-op in sim)
            BAUD b      — baud rate change (no-op in sim)
            STAGE       — stage query (returns identifier)
            I           — stop (immediate)
        """
        if self.communication_delay > 0:
            time.sleep(self.communication_delay)

        command = command.strip()

        # Velocity command: VS,vx,vy
        if command.startswith("VS"):
            return self._handle_velocity(command)
        # Absolute move: PA,x,y (internal format)
        if command.startswith("PA"):
            return self._handle_absolute_pa(command)
        # Absolute move: G x,y (ProScan format)
        if command.startswith("G ") or command.startswith("G,"):
            return self._handle_absolute_g(command)
        # Relative move: GR dx,dy
        if command.startswith("GR"):
            return self._handle_relative(command)
        # Position query
        if command == "P":
            return self._handle_position_query()
        # Firmware version
        if command == "V":
            return "Prior ProScan III Simulator v7.1.2"
        # Set home
        if command == "Z":
            return self._handle_set_home()
        # Max speed
        if command.startswith("SMS"):
            return self._handle_set_speed(command)
        # Acceleration
        if command.startswith("SAS"):
            return self._handle_set_acceleration(command)
        # Jerk (no-op)
        if command.startswith("SCS"):
            return "R"
        # Stop (immediate)
        if command == "I":
            return self._handle_stop()
        # Baud rate (no-op)
        if command.startswith("BAUD"):
            return "R"
        # Stage query
        if command == "STAGE":
            return "ProScan III Simulator"

        logger.debug(f"Unknown simulator command: {command}")
        return "E"

    # ── Command Handlers ──────────────────────────────────────────

    def _handle_velocity(self, command: str) -> str:
        """Handle 'VS,vx,vy' velocity command."""
        parts = command.split(",")
        if len(parts) != 3:
            return "E"
        try:
            vx = float(parts[1])
            vy = float(parts[2])
        except ValueError:
            return "E"
        with self._lock:
            self.mode = "velocity"
            self.target_vx = max(-self.max_speed, min(self.max_speed, vx))
            self.target_vy = max(-self.max_speed, min(self.max_speed, vy))
        return "R"

    def _handle_absolute_pa(self, command: str) -> str:
        """Handle 'PA,x,y' absolute move (internal format)."""
        parts = command.split(",")
        if len(parts) != 3:
            return "E"
        try:
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            return "E"
        with self._lock:
            self.mode = "absolute"
            self.target_x = x
            self.target_y = y
        return "R"

    def _handle_absolute_g(self, command: str) -> str:
        """Handle 'G x,y' absolute move (ProScan format)."""
        # Format: "G x,y" or "G x,y\r"
        payload = command[2:].strip()
        parts = payload.split(",")
        if len(parts) != 2:
            return "E"
        try:
            x = float(parts[0])
            y = float(parts[1])
        except ValueError:
            return "E"
        with self._lock:
            self.mode = "absolute"
            self.target_x = x
            self.target_y = y
        return "R"

    def _handle_relative(self, command: str) -> str:
        """
        Handle 'GR dx,dy' relative move.

        CRITICAL FIX: Accumulates offset onto the TARGET position, not the
        current (moving) position. When rapid GR commands arrive in sequence:

            Old (buggy):  target = current_pos + dx  → loses accumulated offset
            New (correct): target = target_pos + dx  → properly accumulates

        If already in velocity mode, switches to absolute and uses current
        position as the base.
        """
        payload = command[3:].strip()
        parts = payload.split(",")
        if len(parts) != 2:
            return "E"
        try:
            dx = float(parts[0])
            dy = float(parts[1])
        except ValueError:
            return "E"
        with self._lock:
            if self.mode == "absolute":
                # Already moving toward a target — accumulate on the target
                self.target_x += dx
                self.target_y += dy
            else:
                # Was in velocity mode — switch to absolute from current position
                self.mode = "absolute"
                self.target_x = self.current_x + dx
                self.target_y = self.current_y + dy
        return "R"

    def _handle_position_query(self) -> str:
        with self._lock:
            return f"{self.current_x:.2f},{self.current_y:.2f},0.00"

    def _handle_set_home(self) -> str:
        with self._lock:
            self.current_x = 0.0
            self.current_y = 0.0
            self.target_x = 0.0
            self.target_y = 0.0
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.target_vx = 0.0
            self.target_vy = 0.0
        return "R"

    def _handle_set_speed(self, command: str) -> str:
        parts = command.split(",")
        if len(parts) == 2:
            try:
                self.max_speed = float(parts[1])
            except ValueError:
                return "E"
        return "R"

    def _handle_set_acceleration(self, command: str) -> str:
        """Handle 'SAS,accel' acceleration command."""
        parts = command.split(",")
        if len(parts) == 2:
            try:
                self.acceleration_rate = float(parts[1])
            except ValueError:
                return "E"
        return "R"

    def _handle_stop(self) -> str:
        """Handle 'I' immediate stop command."""
        with self._lock:
            self.mode = "velocity"
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.current_vx = 0.0
            self.current_vy = 0.0
        return "R"

    # ── Position Query (direct, for convenience) ──────────────────

    def get_current_position(self) -> tuple[float, float, float]:
        """Return (x, y, 0.0) — thread-safe direct access."""
        with self._lock:
            return self.current_x, self.current_y, 0.0

    # ── Physics Loop ──────────────────────────────────────────────

    def _ramp_velocity(self, current: float, target: float, dt: float) -> float:
        """Linear velocity ramp toward target with acceleration limit."""
        max_change = self.acceleration_rate * dt
        if current < target:
            return min(current + max_change, target)
        elif current > target:
            return max(current - max_change, target)
        return current

    def _update_loop(self) -> None:
        """Continuous physics update loop."""
        while self._running:
            start = time.time()

            with self._lock:
                dt = start - self._last_update_time
                self._last_update_time = start

                # Prevent huge dt jumps (e.g., system sleep)
                dt = min(dt, 0.1)

                if self.mode == "absolute":
                    # Proportional controller toward target position
                    error_x = self.target_x - self.current_x
                    error_y = self.target_y - self.current_y

                    # If within settling threshold, snap to target and stop
                    if abs(error_x) < self.settling_threshold and abs(error_y) < self.settling_threshold:
                        self.current_x = self.target_x
                        self.current_y = self.target_y
                        self.current_vx = 0.0
                        self.current_vy = 0.0
                    else:
                        desired_vx = max(-self.max_speed, min(self.max_speed, self.kp * error_x))
                        desired_vy = max(-self.max_speed, min(self.max_speed, self.kp * error_y))

                        self.current_vx = self._ramp_velocity(self.current_vx, desired_vx, dt)
                        self.current_vy = self._ramp_velocity(self.current_vy, desired_vy, dt)

                        self.current_x += self.current_vx * dt
                        self.current_y += self.current_vy * dt
                else:
                    # Velocity mode
                    desired_vx = self.target_vx
                    desired_vy = self.target_vy

                    self.current_vx = self._ramp_velocity(self.current_vx, desired_vx, dt)
                    self.current_vy = self._ramp_velocity(self.current_vy, desired_vy, dt)

                    self.current_x += self.current_vx * dt
                    self.current_y += self.current_vy * dt

            elapsed = time.time() - start
            sleep_time = max(0.0, self._update_interval - elapsed)
            time.sleep(sleep_time)
