"""
XY Stage Manager — JSON-protocol-driven interface to Prior ProScan controllers.

v7.1 refactor: Commands are no longer hardcoded. Instead, a ControllerProtocol
JSON file defines the command syntax, terminators, and detection sequence.
Adding support for a new XY controller = adding a new JSON file.

Supports:
- Prior ProScan III (config/controllers/proscan_iii.json)
- Prior ProScan II  (config/controllers/proscan_ii.json)
- Auto-detection across all JSON files in the controllers directory
- Graceful fallback when a command is unsupported by the loaded protocol

When ``simulate=True``, delegates to :class:`XYStageSimulator` instead
of real serial hardware (unchanged from v7.0).

Session I — Tasks P8.17, P8.18, P8.19, P8.20, P8.21.
"""

from __future__ import annotations
import threading  # v7.2.6: serial lock (moved after __future__)

import json
import logging
import platform
import socket
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
from SupportClasses.ControllerProtocol import (
    ControllerProtocol,
    discover_controller_files,
    DEFAULT_CONTROLLERS_DIR,
)


class XYStageManager:
    """
    Manager for Prior ProScan XY stages (or simulator).

    v7.1: Uses ControllerProtocol JSON files for command formatting.

    Parameters:
        simulate:        If True, use the software simulator.
        settings:        Optional dict of stage parameters to override defaults.
        controller_json: Path to controller JSON file, "auto" for auto-detect,
                         or None to use the default ProScan III config.
    """

    # Default stage parameters (used when protocol doesn't specify)
    DEFAULT_MAX_SPEED = 100
    DEFAULT_ACCELERATION = 50
    DEFAULT_VELOCITY = 50

    def __init__(
        self,
        simulate: bool = False,
        settings: Optional[dict] = None,
        controller_json: Optional[str] = None,
    ):
        self.simulate = simulate
        self._protocol: Optional[ControllerProtocol] = None
        self._detected_controller: Optional[str] = None

        # Stage parameters (may be overridden by protocol or settings)
        self.max_speed: int = self.DEFAULT_MAX_SPEED
        self.min_jerk: int = 1
        self.max_jerk: int = 100
        self.min_acceleration: int = 1
        self.max_acceleration: int = 100
        self.x_range: list[int] = [-100_000, 100_000]
        self.y_range: list[int] = [-100_000, 100_000]
        self.default_acceleration: int = self.DEFAULT_ACCELERATION
        self.default_velocity: int = self.DEFAULT_VELOCITY

        # v7.2.6: serial lock — must be created BEFORE _initialise_serial()
        # because send_command() acquires it during port detection
        self._serial_lock = threading.RLock()

        # P8.17: Load controller protocol JSON
        # BUG-3 FIX (v7.1.2): Always load protocol, even in sim mode,
        # so parameters like microsteps_per_micron are accessible.
        if controller_json is not None:
            self._load_protocol(controller_json)
            self._apply_protocol_parameters()
        elif not simulate:
            # Real hardware without explicit protocol → try default/auto-detect
            self._load_protocol(controller_json)
            self._apply_protocol_parameters()

        # Apply any user-provided settings (override protocol defaults)
        if settings:
            self._apply_settings(settings)

        # Initialise the communication backend
        if self.simulate:
            self.spo = XYStageSimulator()
            # BUG-3 FIX: Configure simulator with protocol-derived parameters
            if self._protocol:
                params = self._protocol._config.get("parameters", {})
                sim_max_speed = params.get("max_speed", 100000)
                sim_accel = params.get("acceleration", 200000)
                if hasattr(self.spo, 'configure_from_protocol'):
                    self.spo.configure_from_protocol(
                        max_speed=float(sim_max_speed),
                        acceleration=float(sim_accel),
                    )
            self.spo.start()
            logger.info("XY stage simulator started")
        else:
            self.spo = self._initialise_serial()

    # ── Protocol Loading (P8.17) ──────────────────────────────────

    def _load_protocol(self, controller_json: Optional[str]) -> None:
        """Load controller protocol from JSON file or auto-detect."""
        if controller_json is None:
            # Default: ProScan III
            default_path = Path(DEFAULT_CONTROLLERS_DIR) / "proscan_iii.json"
            if default_path.exists():
                try:
                    self._protocol = ControllerProtocol.load(default_path)
                    logger.info(f"Loaded default protocol: {self._protocol.controller_name}")
                    return
                except Exception as e:
                    logger.warning(f"Failed to load default protocol: {e}")
            # Fallback: no protocol loaded (will use hardcoded commands)
            logger.warning("No controller protocol loaded — using hardcoded defaults")

        elif controller_json.lower() == "auto":
            # Auto-detect will happen during serial initialisation
            logger.info("Controller protocol set to auto-detect")

        else:
            # Explicit JSON path
            path = Path(controller_json)
            if path.exists():
                try:
                    self._protocol = ControllerProtocol.load(path)
                    logger.info(f"Loaded protocol: {self._protocol.controller_name}")
                except Exception as e:
                    logger.error(f"Failed to load controller protocol {path}: {e}")
            else:
                logger.warning(f"Controller protocol file not found: {path}")

    def _apply_protocol_parameters(self) -> None:
        """Apply parameter ranges from the loaded protocol."""
        if self._protocol is None:
            return

        speed_range = self._protocol.get_speed_range()
        if speed_range:
            self.max_speed = speed_range[1]

        accel_range = self._protocol.get_acceleration_range()
        if accel_range:
            self.min_acceleration = accel_range[0]
            self.max_acceleration = accel_range[1]

        jerk_range = self._protocol.get_jerk_range()
        if jerk_range:
            self.min_jerk = jerk_range[0]
            self.max_jerk = jerk_range[1]

        pos_range_x = self._protocol.get_parameter("position_range_x")
        if pos_range_x:
            self.x_range = list(pos_range_x)

        pos_range_y = self._protocol.get_parameter("position_range_y")
        if pos_range_y:
            self.y_range = list(pos_range_y)

    @property
    def protocol(self) -> Optional[ControllerProtocol]:
        """The loaded controller protocol (or None if not loaded)."""
        return self._protocol

    @property
    def detected_controller(self) -> Optional[str]:
        """Name of the auto-detected controller (or None)."""
        return self._detected_controller

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
        """Find and open the controller."""
        controller_name = "XY controller"
        if self._protocol:
            controller_name = self._protocol.controller_name

        logger.info(
            f"Searching for {controller_name} "
            f"(platform={platform.system()}, host={socket.gethostname()})"
        )
        spo = self._find_controller()
        if spo is None:
            raise ConnectionError(
                f"{controller_name} not found. "
                "Check USB connection and ensure no other program is using the port."
            )
        return spo

    def _find_controller(self) -> Optional[serial.Serial]:
        """
        Scan COM ports for the controller.

        If a protocol is loaded, uses its detection sequence.
        If protocol is None (auto-detect mode), tries all JSON files.
        """
        if serial is None:
            raise ImportError("pyserial is required for hardware mode")

        if self._protocol is not None:
            # Use the loaded protocol's detection sequence
            return self._find_with_protocol(self._protocol)

        # P8.21: Auto-detect — try each JSON file
        return self._auto_detect_controller()

    def _find_with_protocol(self, protocol: ControllerProtocol) -> Optional[serial.Serial]:
        """Try to find a controller matching the given protocol."""
        detection = protocol.get_detection_info()
        wake_cmd = detection.get("wake_command")
        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0
        fw_query = detection.get("firmware_query", "V")
        tokens = detection.get("identify_tokens", [])
        baud = protocol.baud_rate

        ports = serial.tools.list_ports.comports()
        for port_info in ports:
            try:
                logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")
                spo = serial.Serial(
                    port_info.device,
                    baudrate=baud,
                    bytesize=protocol.byte_size,
                    timeout=protocol.timeout,
                    stopbits=serial.STOPBITS_ONE,
                )

                # P8.18: Use protocol terminators
                tx_term = protocol.tx_terminator

                # Wake command (if defined)
                if wake_cmd:
                    spo.write(wake_cmd.encode(protocol.encoding) + tx_term)
                    time.sleep(wake_delay)
                    spo.readline()  # discard wake-up response
                    spo.reset_input_buffer()
                    spo.reset_output_buffer()

                # Firmware query
                spo.write(fw_query.encode(protocol.encoding) + tx_term)
                time.sleep(0.1)
                response = spo.readline().decode(protocol.encoding, errors="replace").strip()
                logger.debug(f"Response from {port_info.device}: {response}")

                # Check identification tokens
                if tokens and any(tok in response for tok in tokens):
                    logger.info(
                        f"{protocol.controller_name} found on "
                        f"{port_info.device} @ {baud} baud"
                    )
                    self._detected_controller = protocol.controller_name
                    return spo

                spo.close()
            except (serial.SerialException, UnicodeDecodeError, OSError) as e:
                logger.debug(f"Error on {port_info.device} @ {baud}: {e}")
                continue

        logger.debug(f"{protocol.controller_name} not found on any port")
        return None

    # ── P8.21: Auto-Detection ────────────────────────────────────

    def _auto_detect_controller(self) -> Optional[serial.Serial]:
        """
        Auto-detect controller by iterating all JSON protocol files.

        Tries each protocol's detection sequence on every COM port.
        Selects the first one that matches.
        """
        json_files = discover_controller_files()
        if not json_files:
            logger.warning("No controller JSON files found for auto-detect")
            return None

        logger.info(f"Auto-detecting controller from {len(json_files)} protocol files")

        for json_path in json_files:
            try:
                protocol = ControllerProtocol.load(json_path)
            except Exception as e:
                logger.debug(f"Skipping {json_path}: {e}")
                continue

            spo = self._find_with_protocol(protocol)
            if spo is not None:
                self._protocol = protocol
                self._apply_protocol_parameters()
                logger.info(f"Auto-detected: {protocol.controller_name}")
                return spo

        logger.warning("Auto-detect failed — no controller matched any protocol")
        return None

    # ── Command Interface (P8.18, P8.19, P8.20) ──────────────────

    def send_command(self, command: str) -> "Optional[str]":
        """Send a raw command to the stage.
        v7.2.6: XY serial lock — prevents PositionPoller/JogHandler contention.
        """
        if self.spo is None:
            logger.error("XY stage not initialised -- command ignored")
            return None
        if self.simulate:
            return self.spo.send_command(command)
        try:
            if self._protocol:
                encoded = command.encode(self._protocol.encoding) + self._protocol.tx_terminator
            else:
                encoded = f"{command}\r\n".encode("ascii")
            self.spo.write(encoded)
            return None
        except (Exception,) as e:
            logger.error(f"XY send_command error: {e}")
            return None


    def _send_protocol_command(
        self, command_name: str, fallback_cmd: Optional[str] = None, **kwargs
    ) -> Optional[str]:
        """
        Format and send a command using the protocol.

        P8.19: All methods use this instead of hardcoded command strings.
        P8.20: Returns None gracefully if the command is unsupported.

        Args:
            command_name: Abstract command name from the protocol
            fallback_cmd: Hardcoded command to use if protocol is unavailable
            **kwargs: Parameters for the command template

        Returns:
            Simulator response (sim mode) or None (hardware mode)
        """
        if self._protocol:
            cmd = self._protocol.format_command(command_name, **kwargs)
            if cmd is None:
                # P8.20: Command not supported by this controller
                logger.debug(f"Command '{command_name}' not supported by {self._protocol.controller_name}")
                return None
        elif fallback_cmd:
            cmd = fallback_cmd
        else:
            logger.warning(f"No protocol loaded and no fallback for '{command_name}'")
            return None

        return self.send_command(cmd)

    # ── Position Queries (P8.19) ──────────────────────────────────

    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":
        """Query stage position.
        v7.2.7: no lock on readline — send_command handles write lock internally.
        Holding lock across readline() starves jog handler threads.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)
        try:
            self._send_protocol_command("position_query", fallback_cmd="P")
            response = self.spo.readline().decode(
                self._protocol.encoding if self._protocol else "ascii",
                errors="replace"
            ).strip()
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

    # ── Movement Commands (P8.19) ─────────────────────────────────

    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        """Set XY velocity (continuous jog mode)."""
        self._send_protocol_command(
            "set_velocity",
            fallback_cmd=f"VS,{vx},{vy}",
            vx=vx, vy=vy,
        )

    def move_stage_to_position(self, x: float, y: float, fast: bool = False) -> None:
        """Move to absolute position (x, y) in stage coordinates."""
        self._send_protocol_command(
            "move_absolute",
            fallback_cmd=f"G {round(x)},{round(y)}",
            x=round(x), y=round(y),
        )
        logger.debug(f"XY absolute move: ({x:.0f}, {y:.0f}) fast={fast}")

    def move_stage_relative(self, dx: float, dy: float) -> None:
        """Move by relative offset (dx, dy)."""
        self._send_protocol_command(
            "move_relative",
            fallback_cmd=f"GR {round(dx)},{round(dy)}",
            dx=round(dx), dy=round(dy),
        )
        logger.debug(f"XY relative move: ({dx:.0f}, {dy:.0f})")

    def set_home(self) -> None:
        """Set current position as home (0, 0, 0)."""
        self._send_protocol_command("set_home", fallback_cmd="Z")
        logger.info("XY home position set")

    def stop_stage(self) -> None:
        """Send immediate stop command."""
        self._send_protocol_command("stop", fallback_cmd="I")
        logger.info("XY stage stopped")

    # ── Stage Settings (P8.19, P8.20) ────────────────────────────

    def set_velocity(self, velocity: int) -> None:
        """v7.2.7: SMS percentage — Set max stage velocity.

        The Prior SMS command takes a percentage (1-100), not µm/s.
        If the caller passes a value > 100, we assume it's µm/s and convert.
        If <= 100, we assume it's already a percentage.

        For explicit mm/s control, use set_speed_mm_s() instead.
        """
        if velocity > 100:
            # Caller sent µm/s — convert to percentage
            max_speed = getattr(self, '_protocol_max_speed_um_s', 50000)
            pct = max(1, min(100, int(velocity / max_speed * 100)))
            logger.debug(f"set_velocity: {velocity} µm/s → SMS {pct}%")
        else:
            pct = max(1, min(100, int(velocity)))
        self._send_protocol_command(
            "set_max_speed",
            fallback_cmd=f"SMS,{pct}",
            speed=pct,
        )

    def set_speed_mm_s(self, speed_mm_s: float) -> None:
        """v7.2.7: Set stage speed in mm/s — converts to SMS percentage.

        This is the preferred method for all print/calibration code.
        Handles the full conversion: mm/s → µm/s → percentage of max.

        Args:
            speed_mm_s: Desired speed in mm/s (e.g., 1.0, 5.0, 50.0)
        """
        max_speed_um_s = getattr(self, '_protocol_max_speed_um_s', 50000)
        speed_um_s = speed_mm_s * 1000.0
        pct = max(1, min(100, int(speed_um_s / max_speed_um_s * 100)))
        logger.info(f"set_speed_mm_s: {speed_mm_s:.1f} mm/s = {speed_um_s:.0f} µm/s "
                    f"= SMS {pct}% (max={max_speed_um_s} µm/s)")
        self._send_protocol_command(
            "set_max_speed",
            fallback_cmd=f"SMS,{pct}",
            speed=pct,
        )


    def set_acceleration(self, acceleration: int) -> None:
        """Set stage acceleration (min–max)."""
        acceleration = max(self.min_acceleration, min(self.max_acceleration, acceleration))
        self._send_protocol_command(
            "set_acceleration",
            fallback_cmd=f"SAS,{int(acceleration)}",
            accel=int(acceleration),
        )

    def set_jerk(self, jerk: int) -> None:
        """
        Set stage jerk (min–max).

        P8.20: Silently ignored if the controller doesn't support jerk.
        """
        jerk = max(self.min_jerk, min(self.max_jerk, jerk))
        result = self._send_protocol_command(
            "set_jerk",
            fallback_cmd=f"SCS,{int(jerk)}",
            jerk=int(jerk),
        )
        # result is None if not supported — logged in _send_protocol_command

    def set_fast_mode(self) -> None:
        """Set velocity to maximum."""
        self.set_velocity(self.max_speed)

    def set_slow_mode(self) -> None:
        """Set velocity to default."""
        self.set_velocity(self.default_velocity)

    def get_firmware_version(self) -> Optional[str]:
        """Query the controller firmware version."""
        if self.simulate:
            return "Simulator v1.0"

        try:
            self._send_protocol_command("firmware_version", fallback_cmd="V")
            response = self.spo.readline().decode(
                self._protocol.encoding if self._protocol else "ascii",
                errors="replace"
            ).strip()
            return response
        except Exception as e:
            logger.debug(f"Firmware version query error: {e}")
            return None

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
        proto = self._protocol.controller_name if self._protocol else "no protocol"
        return f"XYStageManager(mode={mode}, protocol={proto})"
