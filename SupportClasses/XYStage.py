"""
XY Stage Manager — Interface to Prior ProScan III XY stage.

Handles serial discovery, command formatting, and position parsing.
When ``simulate=True``, delegates to :class:`XYStageSimulator` instead
of real serial hardware.

Common ProScan III commands:
    V           Query firmware version
    Z           Set current position as home (0,0,0)
    P           Query current position → "x,y,z"
    G x,y       Absolute move to (x, y)
    GR dx,dy    Relative move by (dx, dy)
    VS,vx,vy    Set velocity
    SMS,speed   Set max speed
    SAS,accel   Set acceleration
    SCS,jerk    Set jerk
"""

from __future__ import annotations

import json
import logging
import platform
import socket
import sys
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore[assignment]
    logger.warning("pyserial not installed — hardware mode unavailable")

from SupportClasses.XYStageSimulator import XYStageSimulator


class XYStageManager:
    """
    Manager for the Prior ProScan III XY stage (or simulator).

    Parameters:
        simulate:  If True, use the software simulator.
        settings:  Optional dict of stage parameters to override defaults.
    """

    # Default stage parameters
    DEFAULT_MAX_SPEED = 100
    DEFAULT_ACCELERATION = 50
    DEFAULT_VELOCITY = 50
    DEFAULT_BAUD_RATES = [38400]
    POSITION_RANGE_X = (-100000, 100000)
    POSITION_RANGE_Y = (-100000, 100000)

    def __init__(self, simulate: bool = False, settings: Optional[dict] = None):
        self.simulate = simulate

        # Stage parameters (may be overridden by settings file)
        self.max_speed: int = self.DEFAULT_MAX_SPEED
        self.min_jerk: int = 1
        self.max_jerk: int = 100
        self.min_acceleration: int = 1
        self.max_acceleration: int = 100
        self.x_range = list(self.POSITION_RANGE_X)
        self.y_range = list(self.POSITION_RANGE_Y)
        self.default_acceleration: int = self.DEFAULT_ACCELERATION
        self.default_velocity: int = self.DEFAULT_VELOCITY

        # Apply any provided settings
        if settings:
            self._apply_settings(settings)

        # Initialise the communication backend
        if self.simulate:
            self.spo = XYStageSimulator()
            self.spo.start()
            logger.info("XY stage simulator started")
        else:
            self.spo = self._initialise_serial()

    # ── Lifecycle ─────────────────────────────────────────────────

    def stop(self) -> None:
        """Disconnect and release resources."""
        if self.spo is None:
            return
        try:
            if self.simulate:
                self.spo.stop()
            else:
                self.spo.close()
        except Exception as e:
            logger.warning(f"Error closing XY stage: {e}")
        logger.info("XY stage stopped")

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    # ── Serial Initialisation ─────────────────────────────────────

    def _initialise_serial(self) -> serial.Serial:
        """Find and open the ProScan III controller."""
        logger.info(
            f"Searching for ProScan III controller "
            f"(platform={platform.system()}, host={socket.gethostname()})"
        )
        spo = self._find_proscan_controller()
        if spo is None:
            raise ConnectionError(
                "ProScan III controller not found. "
                "Check USB connection and ensure no other program is using the port."
            )
        return spo

    def _find_proscan_controller(self) -> Optional[serial.Serial]:
        """Scan COM ports for a ProScan III controller."""
        if serial is None:
            raise ImportError("pyserial is required for hardware mode")

        ports = serial.tools.list_ports.comports()
        for port_info in ports:
            for baud in self.DEFAULT_BAUD_RATES:
                try:
                    logger.debug(f"Trying {port_info.device} @ {baud} baud")
                    spo = serial.Serial(
                        port_info.device,
                        baudrate=baud,
                        bytesize=8,
                        timeout=1,
                        stopbits=serial.STOPBITS_ONE,
                    )

                    # Wake up the controller
                    spo.write(b"STAGE\r\n")
                    time.sleep(0.1)
                    spo.readline()  # discard wake-up response
                    spo.reset_input_buffer()
                    spo.reset_output_buffer()

                    # Check firmware version
                    spo.write(b"V\r\n")
                    time.sleep(0.1)
                    response = spo.readline().decode("ascii", errors="replace").strip()
                    logger.debug(f"Response from {port_info.device}: {response}")

                    if any(tok in response for tok in ("E", "R", "ProScan")):
                        logger.info(
                            f"ProScan III found on {port_info.device} @ {baud} baud"
                        )
                        return spo

                    spo.close()
                except (serial.SerialException, UnicodeDecodeError, OSError) as e:
                    logger.debug(f"Error on {port_info.device} @ {baud}: {e}")
                    continue

        logger.warning("No ProScan III controller found on any port")
        return None

    # ── Command Interface ─────────────────────────────────────────

    def send_command(self, command: str) -> Optional[str]:
        """
        Send a raw command string to the stage.

        In simulation mode, returns the simulator's response directly.
        In hardware mode, writes to serial (response must be read separately).
        """
        if self.spo is None:
            logger.error("XY stage not initialised — command ignored")
            return None

        if self.simulate:
            return self.spo.send_command(command)

        try:
            encoded = f"{command}\r\n".encode("ascii")
            self.spo.write(encoded)
            return None  # caller reads response via get_current_position etc.
        except (serial.SerialException, OSError) as e:
            logger.error(f"XY send_command error: {e}")
            return None

    # ── Position Queries ──────────────────────────────────────────

    def get_current_position(self) -> tuple[float | None, float | None, float | None]:
        """
        Query the stage for its current position.

        Returns:
            (x, y, z) tuple, or (None, None, None) on failure.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)

        try:
            self.send_command("P")
            response = self.spo.readline().decode("ascii", errors="replace").strip()
            return self._parse_position_response(response)
        except Exception as e:
            logger.debug(f"XY position query error: {e}")
            return (None, None, None)

    @staticmethod
    def _parse_position_response(response: str) -> tuple[float | None, float | None, float | None]:
        """Parse 'x,y,z' position response string."""
        try:
            values = response.split(",")
            if len(values) != 3:
                raise ValueError(f"Expected 3 values, got {len(values)}: {response}")
            x, y, z = (
                float(v.strip().replace("\r", "").strip("R")) for v in values
            )
            return x, y, z
        except (ValueError, AttributeError) as e:
            logger.debug(f"Failed to parse XY position: {e}")
            return (None, None, None)

    # ── Movement Commands ─────────────────────────────────────────

    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        """Set XY velocity (continuous jog mode)."""
        self.send_command(f"VS,{vx},{vy}")

    def move_stage_to_position(self, x: float, y: float, fast: bool = False) -> None:
        """Move to absolute position (x, y) in stage coordinates."""
        cmd = f"G {int(x)},{int(y)}"
        self.send_command(cmd)
        logger.debug(f"XY absolute move: ({x:.0f}, {y:.0f}) fast={fast}")

    def move_stage_relative(self, dx: float, dy: float) -> None:
        """Move by relative offset (dx, dy)."""
        cmd = f"GR {int(dx)},{int(dy)}"
        self.send_command(cmd)
        logger.debug(f"XY relative move: ({dx:.0f}, {dy:.0f})")

    def set_home(self) -> None:
        """Set current position as home (0, 0, 0)."""
        self.send_command("Z")
        logger.info("XY home position set")

    # ── Stage Settings ────────────────────────────────────────────

    def set_velocity(self, velocity: int) -> None:
        """Set maximum stage velocity (1–100)."""
        velocity = max(0, min(self.max_speed, velocity))
        self.send_command(f"SMS,{int(velocity)}")

    def set_acceleration(self, acceleration: int) -> None:
        """Set stage acceleration (1–100)."""
        acceleration = max(self.min_acceleration, min(self.max_acceleration, acceleration))
        self.send_command(f"SAS,{int(acceleration)}")

    def set_jerk(self, jerk: int) -> None:
        """Set stage jerk (1–100)."""
        jerk = max(self.min_jerk, min(self.max_jerk, jerk))
        self.send_command(f"SCS,{int(jerk)}")

    def set_fast_mode(self) -> None:
        """Set velocity to maximum."""
        self.set_velocity(self.max_speed)

    def set_slow_mode(self) -> None:
        """Set velocity to default."""
        self.set_velocity(self.default_velocity)

    def check_stage_limits(self, x: float, y: float) -> bool:
        """Return True if (x, y) is within the stage's physical range."""
        return (
            self.x_range[0] <= x <= self.x_range[1]
            and self.y_range[0] <= y <= self.y_range[1]
        )

    # ── Settings Persistence ──────────────────────────────────────

    def _apply_settings(self, settings: dict) -> None:
        """Apply stage settings from a dict."""
        self.max_speed = settings.get("maxSpeed", self.max_speed)
        self.min_jerk = settings.get("minJerk", self.min_jerk)
        self.max_jerk = settings.get("maxJerk", self.max_jerk)
        self.min_acceleration = settings.get("minAcceleration", self.min_acceleration)
        self.max_acceleration = settings.get("maxAcceleration", self.max_acceleration)
        self.x_range = settings.get("xRange", self.x_range)
        self.y_range = settings.get("yRange", self.y_range)
        self.default_acceleration = settings.get("defaultAcceleration", self.default_acceleration)
        self.default_velocity = settings.get("defaultVelocity", self.default_velocity)

    def load_stage_settings(self, filepath: str = "stage_settings.json") -> None:
        """Load stage settings from a JSON file."""
        path = Path(filepath)
        if not path.exists():
            logger.debug(f"Stage settings file not found: {path}")
            return
        try:
            with open(path) as f:
                data = json.load(f)
            stage_data = data.get("PriorIII_StageSettings", data)
            self._apply_settings(stage_data)
            logger.info(f"Stage settings loaded from {path}")
        except Exception as e:
            logger.warning(f"Error loading stage settings: {e}")

    def __repr__(self) -> str:
        mode = "SIM" if self.simulate else "HW"
        return f"XYStageManager(mode={mode})"
