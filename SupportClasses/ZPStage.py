"""
ZP Stage Manager — Interface to the Marlin-based Z/Pump stage.

Manages communication with a 3D printer board that controls:
    - Z axis (vertical needle) → mapped to printer X axis
    - P1 syringe pump         → mapped to printer Y axis
    - P2 syringe pump         → mapped to printer Z axis
    - P3 syringe pump         → mapped to printer E axis

When ``simulate=True``, delegates to :class:`ZPStageSimulator`.
"""

from __future__ import annotations

import logging
import threading
import re
import time
from typing import Optional

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore[assignment]
    logger.warning("pyserial not installed — hardware mode unavailable")

from SupportClasses.ZPStageSimulator import ZPStageSimulator


# ═══════════════════════════════════════════════════════════════════
# Logical → Printer Axis Mapping
# ═══════════════════════════════════════════════════════════════════

AXIS_MAP: dict[str, str] = {
    "Z": "X",    # Vertical needle → printer X
    "P1": "Y",   # Syringe pump 1  → printer Y
    "P2": "Z",   # Syringe pump 2  → printer Z
    "P3": "E",   # Syringe pump 3  → printer E
}

AXIS_MAP_REVERSE: dict[str, str] = {v: k for k, v in AXIS_MAP.items()}


class ZPStageManager:
    """
    Manager for the Marlin-based ZP stage (or simulator).

    Parameters:
        simulate:        If True, use the software simulator.
        baudrate:        Serial baud rate for real hardware.
        default_feedrate: Default movement feedrate (mm/min).
        steps_per_mm:    Steps-per-mm for the stepper calibration.
    """

    DEFAULT_BAUDRATE = 38400
    DEFAULT_FEEDRATE = 200      # mm/min
    DEFAULT_STEPS_PER_MM = 5069

    # Position pattern for M114 response parsing
    _M114_PATTERN = re.compile(
        r"X:([+-]?\d+\.?\d*)\s+"
        r"Y:([+-]?\d+\.?\d*)\s+"
        r"Z:([+-]?\d+\.?\d*)\s+"
        r"E:([+-]?\d+\.?\d*)\s+"
        r"Count\s+X:([+-]?\d+)\s+Y:([+-]?\d+)\s+Z:([+-]?\d+)"
    )

    def __init__(
        self,
        simulate: bool = False,
        baudrate: int = DEFAULT_BAUDRATE,
        default_feedrate: float = DEFAULT_FEEDRATE,
        steps_per_mm: int = DEFAULT_STEPS_PER_MM,
    ):
        # v7.2.6: lock before init
        self._serial_lock = threading.RLock()  # v7.2.6: ZP serial lock
        self.simulate = simulate
        # v7.2.6: ZP serial lock — thread-safe serial access
        self.baudrate = baudrate
        self.feedrate = default_feedrate
        self.steps_per_mm = steps_per_mm

        # Cached position values (updated on get_current_position)
        self.x_pos: float = 0.0
        self.y_pos: float = 0.0
        self.z_pos: float = 0.0
        self.e_pos: float = 0.0

        # Initialise communication backend
        if simulate:
            self.serial = ZPStageSimulator()
            self.serial.start()
            self._setup_printer()
            logger.info("ZP stage simulator started")
        else:
            self.serial = self._initialise_serial()
            if self.serial is not None:
                self._setup_printer()

    # ── Lifecycle ─────────────────────────────────────────────────

    def stop(self) -> None:
        """Disconnect and release resources."""
        if self.serial is None:
            return
        try:
            if self.simulate:
                self.serial.stop()
            else:
                self.serial.close()
        except Exception as e:
            logger.warning(f"Error closing ZP stage: {e}")
        logger.info("ZP stage stopped")

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    # ── Serial Initialisation ─────────────────────────────────────

    def _initialise_serial(self) -> Optional[serial.Serial]:
        """Find and open a 3D printer board."""
        if serial is None:
            raise ImportError("pyserial is required for hardware mode")

        ports = self._get_available_ports()
        for port_device in ports:
            if self._is_marlin_printer(port_device):
                logger.info(f"3D printer board found on {port_device}")
                try:
                    ser = serial.Serial(port_device, baudrate=self.baudrate, timeout=1)
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    return ser
                except serial.SerialException as e:
                    logger.error(f"Failed to open {port_device}: {e}")

        logger.warning("No 3D printer board found")
        return None

    @staticmethod
    def _get_available_ports() -> list[str]:
        """List available serial port device names."""
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception as e:
            logger.error(f"Error listing COM ports: {e}")
            return []

    @staticmethod
    def _is_marlin_printer(port: str) -> bool:
        """Probe a port for Marlin firmware via M115."""
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(b"\nM115\n")
                response = ser.read_until(b"\n").decode("utf-8", errors="replace")
                return "FIRMWARE_NAME" in response
        except (serial.SerialException, OSError) as e:
            logger.debug(f"Probe failed on {port}: {e}")
            return False

    # ── Printer Setup ─────────────────────────────────────────────

    def _setup_printer(self) -> None:
        """Configure the printer for bioprinting operation."""
        spm = self.steps_per_mm
        commands = [
            "M302 S0",                          # Allow cold extrusion
            "M83",                               # Extruder relative mode
            "G91",                               # Relative positioning
            f"M203 E{self.feedrate} Y{self.feedrate} X{self.feedrate} Z{self.feedrate}",
            f"M92 X{spm}.00 Y{spm}.00 Z-{spm}.00 E{spm}.00",
            f"G0 F{self.feedrate}",              # Set initial feedrate
            "M220 S100",                         # Speed factor 100%
        ]
        for cmd in commands:
            self.send_data(cmd)
        logger.info(f"ZP stage configured (feedrate={self.feedrate}, steps/mm={spm})")

    # ── Communication ─────────────────────────────────────────────

    def send_data(self, data: str) -> None:
        """Send G-code command to printer.
        v7.2.6: ZP serial lock protects write+flush.
        """
        if self.serial is None:
            logger.error("ZP serial not initialised -- command ignored")
            return
        encoded = data.encode("utf-8") + b"\n"
        for attempt in range(5):
            if hasattr(self.serial, "is_open") and not self.serial.is_open:
                if attempt < 4:
                    import time as _t; _t.sleep(0.01)
                    continue
                logger.error("ZP serial still not open after retries")
                return
            break
        with self._serial_lock:
            try:
                self.serial.write(encoded)
                self.serial.flush()
            except Exception as e:
                logger.error(f"ZP send_data error: {e}")


    def receive_data(self) -> str:
        """Read all available response data.
        v7.2.6: ZP serial lock protects read_all.
        """
        import time as _t; _t.sleep(0.01)
        with self._serial_lock:
            try:
                return self.serial.read_all().decode("utf-8", errors="replace").strip()
            except Exception as e:
                logger.debug(f"ZP receive_data error: {e}")
                return ""


    def move_relative(self, axes: dict[str, float], feedrate: Optional[float] = None) -> None:
        """
        Move axes by relative distances (in relative mode).

        Args:
            axes:     Dict of {printer_axis: distance}, e.g. {"X": 1.5, "Y": -0.5}
            feedrate: Optional feedrate override (mm/min).
        """
        # Filter out zero moves
        active = {a: d for a, d in axes.items() if abs(d) > 1e-6}
        if not active:
            return

        axis_str = " ".join(f"{a}{d}" for a, d in active.items())
        fr = feedrate if feedrate is not None else self.feedrate
        self.send_data(f"G0 F{fr} {axis_str}")

    def move_absolute(self, axes: dict[str, float], fast: bool = False,
                      feedrate_mm_min: float | None = None) -> None:
        """
        Move axes to absolute positions.

        Temporarily switches to absolute mode, executes the move,
        then returns to relative mode.

        Args:
            axes: Dict of {printer_axis: position}
            fast: If True, use maximum feedrate.
            feedrate_mm_min: Optional feedrate for this move (mm/min).
        """
        active = {a: p for a, p in axes.items() if True}  # include all
        if not active:
            return

        axis_str = " ".join(f"{a}{p}" for a, p in active.items())
        feed_str = f" F{feedrate_mm_min:.0f}" if feedrate_mm_min else ""

        self.send_data("G90")  # Absolute mode
        self.send_data(f"G0 {axis_str}{feed_str}")
        self.send_data("G91")  # Back to relative

    # ── Position Query ────────────────────────────────────────────

    def get_current_position(self) -> tuple[float, float, float, float]:
        """
        Query and return current position of all four axes.

        Returns:
            (x, y, z, e) — printer axis positions.
            Maps to (Z-needle, P1, P2, P3) via AXIS_MAP.
        """
        self.send_data("M114")
        response = self.receive_data()
        self._parse_position(response)
        return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)

    def _parse_position(self, response: str) -> None:
        """Parse M114 response and update cached positions."""
        for line in response.split("\n"):
            line = line.strip()
            if not line or line == "ok":
                continue

            match = self._M114_PATTERN.search(line)
            if match:
                self.x_pos = float(match.group(1))
                self.y_pos = float(match.group(2))
                self.z_pos = float(match.group(3))
                self.e_pos = float(match.group(4))
                return

        logger.debug(f"ZP position parse failed: {response[:100]}")

    # ── Settings ──────────────────────────────────────────────────

    def set_max_feedrate(self, feedrate: float) -> None:
        """Set maximum feedrate for all axes."""
        self.feedrate = feedrate
        self.send_data(f"M203 E{feedrate} Y{feedrate} X{feedrate} Z{feedrate}")
        logger.debug(f"ZP max feedrate set to {feedrate} mm/min")

    def set_absolute_mode(self) -> None:
        """Switch to absolute positioning."""
        self.send_data("G90")

    def set_relative_mode(self) -> None:
        """Switch to relative positioning."""
        self.send_data("G91")

    def emergency_stop(self) -> None:
        """Send emergency stop command (M112)."""
        self.send_data("M112")
        logger.warning("ZP emergency stop sent")

    def save_settings(self) -> None:
        """Save current settings to printer EEPROM."""
        self.send_data("M500")

    def __repr__(self) -> str:
        mode = "SIM" if self.simulate else "HW"
        return f"ZPStageManager(mode={mode}, feedrate={self.feedrate})"
