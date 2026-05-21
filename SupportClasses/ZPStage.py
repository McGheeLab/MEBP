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
#
# v7.4.2: AXIS_MAP and steps_per_mm are now instance-level — each
# ZPStageManager can carry its own per-machine mapping. The module-
# level constants below are kept as defaults for backwards compatibility
# and as the canonical starting point for new device profiles.

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
                         v7.4.2: accepts int (back-compat — applied to
                         every logical axis) or dict[str, int] keyed by
                         logical axis (Z, P1, P2, P3). Sign indicates
                         direction; negative inverts.
        axis_map:        v7.4.2: dict mapping logical axes (Z, P1, P2,
                         P3) to physical Marlin axes (X, Y, Z, E).
                         Defaults to the module-level AXIS_MAP.
    """

    DEFAULT_BAUDRATE = 38400
    DEFAULT_FEEDRATE = 200      # mm/min
    DEFAULT_STEPS_PER_MM = 5069

    # v7.4.2: Per-axis default. Note P2 inverts direction (negative) —
    # matches the original hard-coded `Z-{spm}.00` in the M92 command
    # for the original single-machine config.
    DEFAULT_STEPS_PER_MM_DICT = {
        "Z": 5069, "P1": 5069, "P2": -5069, "P3": 5069,
    }

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
        steps_per_mm: int | dict[str, int] = DEFAULT_STEPS_PER_MM,
        axis_map: dict[str, str] | None = None,
        preferred_port: str | None = None,
    ):
        # v7.2.6: lock before init
        self._serial_lock = threading.RLock()  # v7.2.6: ZP serial lock
        self.simulate = simulate
        # v7.2.6: ZP serial lock — thread-safe serial access
        self.baudrate = baudrate
        self.feedrate = default_feedrate
        # v7.4.2 hotfix: cached preferred port from last successful
        # connect — tried first to skip the rediscovery scan.
        self.preferred_port = preferred_port
        # Set after a successful connect so the caller can persist it.
        self.connected_port: str | None = None
        # v7.4.2: per-axis steps_per_mm + configurable axis map
        if isinstance(steps_per_mm, dict):
            self.steps_per_mm = dict(steps_per_mm)
        else:
            # Back-compat: int collapses to dict; preserve P2's
            # legacy negative sign so existing call sites continue to work.
            self.steps_per_mm = dict(self.DEFAULT_STEPS_PER_MM_DICT)
            for k in self.steps_per_mm:
                sign = -1 if self.steps_per_mm[k] < 0 else 1
                self.steps_per_mm[k] = sign * abs(int(steps_per_mm))
        self.axis_map = dict(axis_map) if axis_map else dict(AXIS_MAP)
        # v7.4.2 hotfix: per-logical-axis max accel (M201). Defaults
        # are sensible Marlin defaults; callers can override via
        # set_axis_accelerations().
        self.max_accel: dict[str, float] = {
            "Z": 100.0, "P1": 1000.0, "P2": 1000.0, "P3": 1000.0,
        }

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
    #
    # v7.4.2 hotfix: speed up + correct connect path.
    #
    # Before: every port was probed twice — once at hardcoded 115200
    # to look for "FIRMWARE_NAME", then closed and reopened at
    # self.baudrate (38400) for the real session. Each open triggers
    # an Arduino-class DTR reset (Marlin board reboots ~2s), so the
    # naive loop took ~5-10s on a typical Mac with 3-4 random tty
    # ports (Bluetooth, debug-console, etc.).
    #
    # After:
    #   1. Rank candidate ports — hwid/description hints first
    #      (MARLIN, STM32, ATMEGA, VID:PID hints), then known-good
    #      naming patterns (usbmodem, usbserial, COM), then skip
    #      obvious junk (Bluetooth-*, debug-console, headphones).
    #   2. Try the previously-good port first (preferred_port).
    #   3. Single open per port at self.baudrate — if Marlin
    #      responds to M115, that same handle is the session.
    #   4. Drain Marlin's boot banner before checking the M115
    #      response, so a "start"/"echo:..." line doesn't look
    #      like a failed probe.

    # Platform-agnostic skip patterns. Lowercase substring match
    # against port.device.
    _SKIP_DEVICE_PATTERNS = (
        "bluetooth", "debug-console", "qc35", "headphone", "speaker",
        "incoming-port", "wireless",
    )
    # Lowercase substring match against port.device. Bumps score.
    _LIKELY_DEVICE_PATTERNS = (
        "usbmodem", "usbserial", "ttyusb", "ttyacm",
    )
    # Substring match (case-insensitive) against port.hwid + description.
    # Strong Marlin signal.
    _MARLIN_HWID_HINTS = (
        "MARLIN", "STM32", "ATMEGA", "ARDUINO",
        # Common Marlin board VID:PID — extend as needed
        "0483:5740",   # STM32 CDC
        "1A86:7523",   # CH340G (common Marlin clone boards)
        "0403:6001",   # FT232 (FTDI) — common adapter
        "10C4:EA60",   # CP210x — Prusa/etc.
        "2341:",       # Arduino VID
    )

    def _initialise_serial(self) -> Optional["serial.Serial"]:
        """Find and open the Marlin board (v7.4.2 hotfix: ranked + cached)."""
        if serial is None:
            raise ImportError("pyserial is required for hardware mode")

        try:
            port_infos = list(serial.tools.list_ports.comports())
        except Exception as e:
            logger.error(f"Error listing COM ports: {e}")
            return None

        ranked = self._rank_ports(port_infos, preferred=self.preferred_port)
        if not ranked:
            logger.warning("No serial ports detected")
            return None

        logger.info(
            "ZP probe order: " +
            ", ".join(f"{p.device}({p.description or 'n/a'})" for p in ranked))

        for port_info in ranked:
            device = port_info.device
            device_lc = (device or "").lower()
            if any(s in device_lc for s in self._SKIP_DEVICE_PATTERNS):
                logger.debug(f"Skipping non-candidate port {device}")
                continue

            ser = self._try_open_marlin(device)
            if ser is not None:
                logger.info(f"Marlin board connected on {device}")
                self.connected_port = device
                return ser

        logger.warning("No Marlin board found on any candidate port")
        return None

    @classmethod
    def _rank_ports(cls, port_infos, preferred: str | None = None):
        """Sort serial ports by Marlin-likeness, preferred port first.

        Ranking signals (higher = tried earlier):
          * preferred (last-known-good) port → 10000
          * description/hwid contains a Marlin/STM32/Arduino hint → +500
          * device name matches usbmodem/usbserial/ttyACM* → +100
          * device name matches a known-junk pattern → -1000 (de-facto skipped)
        """
        def score(p) -> int:
            device = (p.device or "").lower()
            hwid = (p.hwid or "").upper()
            desc = (p.description or "").upper()
            s = 0
            if preferred and p.device == preferred:
                s += 10000
            for hint in cls._MARLIN_HWID_HINTS:
                if hint.upper() in hwid or hint.upper() in desc:
                    s += 500
                    break
            for pat in cls._LIKELY_DEVICE_PATTERNS:
                if pat in device:
                    s += 100
                    break
            for skip in cls._SKIP_DEVICE_PATTERNS:
                if skip in device:
                    s -= 1000
                    break
            return s
        return sorted(port_infos, key=score, reverse=True)

    @staticmethod
    def _get_available_ports() -> list[str]:
        """List available serial port device names. v7.2 compat shim."""
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception as e:
            logger.error(f"Error listing COM ports: {e}")
            return []

    def _try_open_marlin(self, port: str,
                         probe_timeout: float = 2.0
                         ) -> Optional["serial.Serial"]:
        """Open ``port`` at self.baudrate, send M115, return the handle
        if Marlin responds with a FIRMWARE_NAME line.

        Single open — the same handle becomes the session if Marlin
        is found. No close-reopen → only one Arduino DTR reset.
        """
        try:
            ser = serial.Serial(port, self.baudrate, timeout=probe_timeout)
        except (serial.SerialException, OSError) as e:
            logger.debug(f"open {port} failed: {e}")
            return None
        try:
            # Brief settle — gives boards that DTR-reset on open a moment
            # to wake up enough to accept bytes. Bigger waits don't help
            # because Marlin's M115 response is what we actually look for.
            time.sleep(0.1)
            # Drain anything in the input buffer (the "start" banner,
            # bootloader chatter, etc.) so it doesn't precede our M115 reply.
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(b"\nM115\n")
            ser.flush()
            # Marlin's M115 response is multiline and terminates with "ok".
            # Read up to a few KB or until we see FIRMWARE_NAME / ok.
            deadline = time.monotonic() + probe_timeout
            accumulated = ""
            while time.monotonic() < deadline:
                chunk = ser.read(512)
                if chunk:
                    try:
                        accumulated += chunk.decode("utf-8", errors="replace")
                    except Exception:
                        accumulated += str(chunk)
                if "FIRMWARE_NAME" in accumulated:
                    logger.debug(f"{port} → Marlin OK: {accumulated[:80]!r}")
                    try:
                        ser.reset_output_buffer()
                    except Exception:
                        pass
                    return ser
                if "ok" in accumulated.lower() and len(accumulated) > 80:
                    # End-of-response without FIRMWARE_NAME → not Marlin
                    break
                if not chunk:
                    # Nothing came back — brief pause before next loop
                    time.sleep(0.05)
            logger.debug(
                f"{port} not Marlin (response: {accumulated[:80]!r})")
            ser.close()
            return None
        except Exception as e:
            logger.debug(f"probe {port} failed: {e}")
            try:
                ser.close()
            except Exception:
                pass
            return None

    @staticmethod
    def _is_marlin_printer(port: str) -> bool:
        """v7.2 compat shim. Kept so existing callers still link.

        v7.4.2 hotfix: ZPStageManager itself no longer goes through
        this method — see ``_try_open_marlin`` which uses
        ``self.baudrate`` consistently instead of the legacy
        hardcoded 115200.
        """
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
        """Configure the printer for bioprinting operation.

        v7.4.2: M92 is now built per-logical-axis from
        ``self.steps_per_mm`` and routed to physical axes via
        ``self.axis_map``. Sign of each value is preserved (negative
        = direction-inverted).
        """
        commands = [
            "M302 S0",                          # Allow cold extrusion
            "M83",                               # Extruder relative mode
            "G91",                               # Relative positioning
            f"M203 E{self.feedrate} Y{self.feedrate} X{self.feedrate} Z{self.feedrate}",
            self._build_m92_command(),
            f"G0 F{self.feedrate}",              # Set initial feedrate
            "M220 S100",                         # Speed factor 100%
        ]
        for cmd in commands:
            self.send_data(cmd)
        logger.info(
            f"ZP stage configured (feedrate={self.feedrate}, "
            f"steps={self.steps_per_mm}, axis_map={self.axis_map})")

    def _build_m92_command(self) -> str:
        """v7.4.2: Build M92 G-code from current steps_per_mm + axis_map.

        Returns e.g. ``"M92 X5069.00 Y5069.00 Z-5069.00 E5069.00"``.
        """
        parts = ["M92"]
        for logical, physical in self.axis_map.items():
            steps = self.steps_per_mm.get(logical, self.DEFAULT_STEPS_PER_MM)
            parts.append(f"{physical}{steps:.2f}")
        return " ".join(parts)

    def set_axis_map(self, axis_map: dict[str, str]) -> None:
        """v7.4.2: Update the logical→physical axis mapping.

        Does NOT re-send M92 — caller should call
        :meth:`set_steps_per_mm(..., persist=True)` afterward if the
        mapping has changed and Marlin needs to be reprogrammed.
        """
        self.axis_map = dict(axis_map)
        logger.info(f"ZP axis_map updated: {self.axis_map}")

    def set_steps_per_mm(self, steps: dict[str, int],
                         persist: bool = True) -> None:
        """v7.4.2: Update per-axis steps_per_mm and optionally send M92.

        Args:
            steps:   dict keyed by logical axis (Z, P1, P2, P3).
            persist: If True (default), send M92 to Marlin so the new
                     calibration takes effect immediately. If False,
                     just updates local state.
        """
        self.steps_per_mm = dict(steps)
        if persist:
            cmd = self._build_m92_command()
            self.send_data(cmd)
            logger.info(f"ZP M92 sent: {cmd}")

    def set_zero(self, logical_axis: str) -> bool:
        """v7.4.2 hotfix: zero the Marlin physical axis mapped to
        ``logical_axis`` by sending a G92.

        Returns True if the command was queued, False if the logical
        axis isn't mapped. After this call, the next M114 query for
        that axis returns 0.
        """
        physical = self.axis_map.get(logical_axis)
        if not physical:
            logger.warning(
                f"set_zero({logical_axis}): no mapping in axis_map={self.axis_map}")
            return False
        self.send_data(f"G92 {physical}0")
        logger.info(f"ZP G92 {physical}0 sent (logical {logical_axis})")
        return True

    def set_axis_accelerations(self, accels: dict[str, float],
                               persist: bool = True) -> None:
        """v7.4.2 hotfix: set per-logical-axis maximum acceleration via M201.

        ``accels`` keys are logical axes (Z, P1, P2, P3). The M201
        command is built from the current axis_map (each logical axis
        is routed to its physical Marlin letter).

        Marlin's M201 expects mm/s²; the command applies to G0/G1
        moves until overridden.
        """
        self.max_accel = dict(accels)
        if persist:
            parts = ["M201"]
            for logical, accel in accels.items():
                physical = self.axis_map.get(logical)
                if physical:
                    parts.append(f"{physical}{float(accel):.2f}")
            cmd = " ".join(parts)
            self.send_data(cmd)
            logger.info(f"ZP {cmd} sent")

    def query_settings(self, timeout: float = 2.0) -> dict:
        """v7.4.2 hotfix: send M503 and parse Marlin's echoed settings.

        Returns a permissive dict with whatever was successfully parsed.
        Example::

            {
                "steps_per_mm": {"X": 5069.0, "Y": 5069.0, "Z": 5070.0, "E": 5069.0},
                "max_feedrate": {"X": 3000.0, "Y": 3000.0, "Z": 600.0, "E": 200.0},
                "max_accel":    {"X": 1000.0, "Y": 1000.0, "Z": 100.0, "E": 1000.0},
            }

        Per-build Marlin output varies; the parser is best-effort —
        unparseable lines are silently ignored.
        """
        result = {"steps_per_mm": {}, "max_feedrate": {}, "max_accel": {}}
        if self.serial is None:
            return result
        try:
            with self._serial_lock:
                try:
                    self.serial.reset_input_buffer()
                except Exception:
                    pass
                self.serial.write(b"M503\n")
                self.serial.flush()
            deadline = time.monotonic() + timeout
            accumulated = ""
            while time.monotonic() < deadline:
                with self._serial_lock:
                    try:
                        chunk = self.serial.read(1024)
                    except Exception:
                        break
                if chunk:
                    try:
                        accumulated += chunk.decode("utf-8", errors="replace")
                    except Exception:
                        accumulated += str(chunk)
                # Stop early if we've seen a long enough echo
                if "ok" in accumulated.lower() and len(accumulated) > 200:
                    break
                if not chunk:
                    time.sleep(0.05)
            self._parse_m503(accumulated, result)
        except Exception as e:
            logger.warning(f"query_settings failed: {e}")
        return result

    _M503_FIELD_RE = re.compile(
        r"\b(?P<letter>[XYZE])(?P<value>[+-]?\d+\.?\d*)")

    def _parse_m503(self, text: str, result: dict) -> None:
        """Parse M503 echo into result dict. Permissive; ignores noise."""
        for line in text.splitlines():
            up = line.strip().upper()
            # Marlin typically prefixes echoes with "echo:" or "echo: "
            up = up.replace("ECHO:", "").strip()
            if up.startswith("M92"):
                bucket = "steps_per_mm"
            elif up.startswith("M203"):
                bucket = "max_feedrate"
            elif up.startswith("M201"):
                bucket = "max_accel"
            else:
                continue
            for m in self._M503_FIELD_RE.finditer(up[3:]):
                letter = m.group("letter")
                try:
                    val = float(m.group("value"))
                except ValueError:
                    continue
                result[bucket][letter] = val

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

    def flush_moves(self, timeout_s: float = 15.0) -> bool:
        """Block until Marlin confirms all queued moves are physically complete.

        Sends M400 (Wait for Moves to Finish). Marlin only responds with 'ok'
        after every buffered move has executed. This is more reliable than
        polling M114, which may return commanded (planned) position rather than
        the actual stepper position during motion.

        Should be called with the PositionPoller suspended to avoid racing on
        the serial port for the 'ok' response.

        v7.3.5 BF-1: Drains the serial RX buffer before sending M400. Prior
        G-code commands (G90, G0, G91 from move_absolute) generate "ok"
        responses that send_data() never reads. Without draining, readline()
        would return a stale "ok" and falsely indicate M400 completion while
        the Z axis is still moving — causing XY to start prematurely.

        Returns:
            True  — Marlin confirmed completion within timeout.
            False — Timed out (caller should log a warning and decide how to proceed).
        """
        if self.serial is None or self.simulate:
            return True  # Simulated or disconnected — treat as immediate success

        # Drain stale "ok" responses and send M400 in a single locked section
        # to prevent any new stale data from arriving between drain and send.
        with self._serial_lock:
            try:
                self.serial.reset_input_buffer()
                self.serial.write(b"M400\n")
                self.serial.flush()
            except Exception as e:
                logger.warning(f"flush_moves send error: {e}")
                return False

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            with self._serial_lock:
                try:
                    line = self.serial.readline().decode("utf-8", errors="replace").strip()
                except Exception as e:
                    logger.warning(f"flush_moves read error: {e}")
                    return False
            if line == "ok":
                return True
            # Discard other lines (position data, temperature reports, etc.)

        logger.warning(f"flush_moves: M400 timed out after {timeout_s:.1f}s")
        return False

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
