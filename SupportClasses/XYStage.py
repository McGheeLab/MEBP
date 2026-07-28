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

import SupportClasses.XYDebugLogger as _dbg

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
import re  # v7.2.8: response_pattern matching


# v7.2.8: CR-aware detection read
def _read_response_cr(spo, timeout=0.3):
    """Read bytes until CR or LF, with short timeout for detection probes.

    pyserial's readline() reads until LF (\\n), but Prior ProScan II
    terminates responses with CR (\\r) only. This causes readline() to
    block until the full timeout (1s+) on every detection probe.
    """
    old_timeout = spo.timeout
    spo.timeout = timeout
    buf = b""
    try:
        while True:
            ch = spo.read(1)
            if not ch:  # timeout
                break
            if ch in (b"\r", b"\n"):
                if buf:  # got data before terminator
                    break
                continue  # skip leading CR/LF
            buf += ch
    except Exception:
        pass
    finally:
        try:
            spo.timeout = old_timeout
        except Exception:
            pass
    return buf.decode("ascii", errors="replace").strip()


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
        exclude_ports: Optional[list] = None,
    ):
        self.simulate = simulate
        self._protocol: Optional[ControllerProtocol] = None
        self._detected_controller: Optional[str] = None
        # v7.5.x: COM ports the detection scan must NEVER open. Opening a port
        # asserts DTR, which auto-RESETS an Arduino/Marlin board (the ZP stage) —
        # so the XY scan is told the sibling ZP port(s) up front and skips them.
        # Normalized to upper-case for case-insensitive matching.
        self._exclude_ports = {str(p).upper() for p in (exclude_ports or []) if p}

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
        # v7.2.8: always load protocol (auto-detect for None/auto,
        # explicit path otherwise). Sim mode also needs protocol
        # for parameters like microsteps_per_micron.
        self._load_protocol(controller_json)
        self._apply_protocol_parameters()

        # Apply any user-provided settings (override protocol defaults)
        if settings:
            self._apply_settings(settings)

        # Initialise the communication backend
        if self.simulate:
            # v7.5.x: pick the simulator by controller family so the Ludl (LEP
            # MAC 5000) wire path — ':A' acks, WHERE, counts — is exercised in
            # sim mode. Prior remains the default (family absent ⇒ "prior").
            family = self._protocol.family if self._protocol else "prior"
            if family == "ludl":
                from SupportClasses.LudlStageSimulator import LudlStageSimulator
                self.spo = LudlStageSimulator()
                if self._protocol:
                    params = self._protocol._config.get("parameters", {})
                    if hasattr(self.spo, 'configure_from_protocol'):
                        self.spo.configure_from_protocol(
                            max_speed=float(params.get("max_speed", 50000)),
                            acceleration=float(params.get("acceleration", 200)),
                            position_scale=float(self._protocol.position_scale),
                        )
            else:
                self.spo = XYStageSimulator()
                # v7.2.9: Ensure SMS percentage conversion uses the simulator's
                # actual MAX_SPEED, not the protocol default (which may differ).
                # Without this, set_speed_mm_s(20) → SMS=40 → only 8 mm/s actual
                # instead of the intended 20 mm/s.
                from SupportClasses.XYStageSimulator import MAX_SPEED_UM_S as _SIM_MAX
                self._protocol_max_speed_um_s = _SIM_MAX
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
            logger.info("XY stage simulator started (%s)", family)
        else:
            self.spo = self._initialise_serial()

    # ── Protocol Loading (P8.17) ──────────────────────────────────

    def _load_protocol(self, controller_json: Optional[str]) -> None:
        """Load controller protocol from JSON file or auto-detect.
        v7.2.8: auto-detect as default when controller_json is None.
        """
        if controller_json is None or controller_json.lower() == "auto":
            # v7.2.8: Auto-detect by default — try all protocols
            # instead of assuming ProScan III. The actual detection
            # happens in _auto_detect_controller() during _find_controller().
            logger.info("Controller protocol set to auto-detect")
            return

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
                # v7.5.x CRASH FIX (twin of ZPStage.stop): serialize close()
                # against any in-flight read under _serial_lock. stop() is
                # called from the watchdog / poller thread on a disconnect,
                # while another thread (poller, or the Stress Test's concurrent
                # XY oscillator → position reads) may be mid-read on this port.
                # Closing a USB-serial handle mid-read is a hard crash on
                # Windows. Bounded acquire so shutdown can't deadlock on a
                # wedged read (Prior reads self-time-out), then close regardless.
                lock = getattr(self, "_serial_lock", None)
                got = lock.acquire(timeout=5.0) if lock is not None else False
                try:
                    self.spo.close()
                finally:
                    if got and lock is not None:
                        lock.release()
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

    def _find_with_protocol(self, protocol: ControllerProtocol) -> "Optional[serial.Serial]":
        """Try to find a controller matching the given protocol.
        v7.2.8s3: Increased settle time + retry on error for reliable detection.
        
        Prior ProScan II returns E,5 ("not initialized") if queried too soon
        after port open, especially when the USB subsystem was churned by
        scanning other ports first. Fix: longer settle + DTR toggle + retry.
        """
        detection = protocol.get_detection_info()
        wake_cmd = detection.get("wake_command")
        wake_delay = detection.get("wake_delay_ms", 100) / 1000.0
        fw_query = detection.get("firmware_query", "V")
        tokens = detection.get("identify_tokens", [])
        resp_pattern = detection.get("response_pattern")
        # v7.5.x: try every candidate baud per port (Prior's own manual warns
        # a changed baud setting can silently revert to a lower default if
        # the port sits idle across power cycles — a real, expected failure
        # mode; scanning bauds here is the documented mitigation). Absent
        # ``communication.baud_rates``, this is a single-element list = the
        # existing ``baud_rate`` — byte-identical to the pre-v7.5.x behavior.
        bauds = protocol.baud_rate_candidates

        ports = serial.tools.list_ports.comports()
        exclude = getattr(self, "_exclude_ports", set())
        for port_info in ports:
            # v7.5.x: never open an excluded port (e.g. the ZP/Marlin board).
            # Opening it asserts DTR → Arduino auto-reset. Skip WITHOUT opening.
            if exclude and str(port_info.device).upper() in exclude:
                logger.debug(f"Skipping excluded port {port_info.device} "
                             f"(reserved for another device)")
                continue
            for baud in bauds:
                try:
                    logger.debug(f"Trying {port_info.device} @ {baud} baud ({protocol.controller_name})")
                    # v7.5.x: stop bits are protocol-driven (Prior=1, LEP MAC 5000=2).
                    stopbits = (serial.STOPBITS_TWO if protocol.stop_bits == 2
                                else serial.STOPBITS_ONE)
                    spo = serial.Serial(
                        port_info.device,
                        baudrate=baud,
                        bytesize=protocol.byte_size,
                        timeout=0.5,
                        stopbits=stopbits,
                    )

                    # v7.2.8s3: DTR toggle + longer settle for USB-serial adapters.
                    # Rapid open/close of other ports can leave the USB subsystem
                    # unsettled, causing E,5 (not initialized) on first query.
                    spo.dtr = False
                    time.sleep(0.05)
                    spo.dtr = True
                    time.sleep(0.3)  # 300ms settle (was 100ms — too short)
                    spo.reset_input_buffer()
                    spo.reset_output_buffer()

                    # v7.5.x: command-mode init bytes (LEP MAC 5000 boots in low-level
                    # binary mode; 0xFF 0x41 forces ASCII/high-level). Sent verbatim,
                    # NO terminator, before any ASCII query. No-op for Prior (absent).
                    init_bytes = protocol.command_mode_init_bytes
                    if init_bytes:
                        spo.write(init_bytes)
                        time.sleep(0.1)
                        spo.reset_input_buffer()

                    tx_term = protocol.tx_terminator

                    # Wake command (if defined)
                    if wake_cmd:
                        spo.write(wake_cmd.encode(protocol.encoding) + tx_term)
                        time.sleep(wake_delay)
                        _read_response_cr(spo, timeout=0.3)
                        spo.reset_input_buffer()

                    # v7.2.8s3: Detection with retry on error.
                    # ProScan returns E,N error codes if not ready.
                    # Retry once after a longer delay.
                    response = ""
                    for attempt in range(2):
                        spo.write(fw_query.encode(protocol.encoding) + tx_term)
                        time.sleep(0.05)
                        response = _read_response_cr(spo, timeout=0.3)
                        logger.debug(
                            f"Detection response from {port_info.device} "
                            f"@ {baud} (attempt {attempt+1}): {response!r}"
                        )

                        if not response:
                            break  # no device here at this baud

                        # If we got an error response (E,N), retry after delay
                        if response.startswith("E,") and attempt == 0:
                            logger.debug(
                                f"{port_info.device}: got error {response!r}, "
                                f"retrying after 500ms settle..."
                            )
                            spo.reset_input_buffer()
                            time.sleep(0.5)
                            continue

                        break  # got a real response (or empty on retry)

                    if not response:
                        spo.close()
                        continue  # next candidate baud

                    # Check response_pattern first (more specific)
                    if resp_pattern:
                        if re.match(resp_pattern, response):
                            logger.info(
                                f"{protocol.controller_name} found on "
                                f"{port_info.device} @ {baud} baud "
                                f"(pattern match: {response!r})"
                            )
                            spo.timeout = protocol.timeout
                            self._detected_controller = protocol.controller_name
                            return spo
                        else:
                            spo.close()
                            continue  # next candidate baud

                    # Fallback: token-based check
                    if tokens and any(tok in response for tok in tokens):
                        logger.info(
                            f"{protocol.controller_name} found on "
                            f"{port_info.device} @ {baud} baud "
                            f"(token match: {response!r})"
                        )
                        spo.timeout = protocol.timeout
                        self._detected_controller = protocol.controller_name
                        return spo

                    spo.close()
                except (serial.SerialException, UnicodeDecodeError, OSError) as e:
                    logger.debug(f"Error on {port_info.device} @ {baud}: {e}")
                    continue  # next candidate baud

        logger.debug(f"{protocol.controller_name} not found on any port")
        return None

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
            # v7.2.8s2: lock is acquired by caller (get_current_position etc.)
            # for atomic write+read. Bare writes (VS, G) don't need response.
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

    # ── v7.5.x: family-neutral wire helpers ───────────────────────

    def _position_scale(self) -> float:
        """Stage wire-units per micron (Prior=1.0, LEP MAC 5000≈10 counts/µm).

        Applied ONLY here on the send/read boundary so every layer above XYStage
        stays µm-native. Prior's 1.0 makes the multiply/divide a no-op.
        """
        return self._protocol.position_scale if self._protocol else 1.0

    def _drain_ack(self, timeout: float = 0.05) -> str:
        """Read + discard one ack line after a write (keeps the poller RX clean).

        Prior replies a bare ``R``; the LEP MAC 5000 replies ``:A`` (ok) or
        ``:N -<code>`` (error). An error ack is logged but NOT raised — the
        historic discipline is drain-and-continue so a stray reply never wedges
        the shared serial poller. The caller must already hold ``_serial_lock``
        (this preserves the pre-v7.5.x inline ``_read_response_cr`` behavior).
        """
        resp = _read_response_cr(self.spo, timeout=timeout)
        if self._protocol is not None:
            m = self._protocol.match_ack_error(resp)
            if m:
                code = m.group(1) if m.groups() else resp
                logger.warning(f"XY controller error ack: {resp!r} (code {code})")
        return resp

    # ── Position Queries (P8.19) ──────────────────────────────────

    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":
        """Query stage position.
        v7.2.8s2: atomic position query — lock protects write+read from
        concurrent JogHandler/PositionPoller contention.
        """
        if self.simulate:
            # v7.2.9: Use direct position read (non-blocking) if available.
            # The serial-sim path send_command("P") blocks during moves,
            # returning a stale snapshot from before the move completed.
            if hasattr(self.spo, 'get_current_position'):
                return self.spo.get_current_position()
            response = self.spo.send_command("P")
            return self._parse_position_response(response)
        try:
            with self._serial_lock:
                # v7.5.0: flush any backlogged/stale RX bytes (orphaned 'R'
                # acks, older position lines) before querying. All other
                # senders drain their own ack synchronously under this same
                # lock, so anything still buffered here is stale by definition.
                # Without this, the poller drains the backlog one CR-line per
                # cycle, making the display "slowly count down" to the real
                # position instead of tracking it.
                try:
                    self.spo.reset_input_buffer()
                except Exception:
                    pass
                self._send_protocol_command("position_query", fallback_cmd="P")
                response = _read_response_cr(self.spo, timeout=0.5)
            result = self._parse_position_response(response)
            _dbg.log("POLL", raw_rx=repr(response),
                     pos_x=result[0], pos_y=result[1], pos_z=result[2])
            return result
        except Exception as e:
            logger.debug(f"XY position query error: {e}")
            _dbg.log("QUERY_ERR", note=str(e))
            return (None, None, None)


    def _parse_position_response(
        self, response: str
    ) -> tuple[float | None, float | None, float | None]:
        """Parse a position response into ``(x, y, z)`` **microns**.

        v7.5.x: protocol-driven. With no protocol loaded (or a Prior JSON that
        omits ``position_parse``) this is byte-identical to the pre-v7.5.x
        staticmethod — comma-split, strip a trailing ``R``, exactly x,y,z. The
        LEP MAC 5000 sets ``position_parse`` to whitespace/2-axis with a ``:A``
        prefix (``WHERE X Y`` → ``:A <x> <y>``). Wire units are divided back to
        µm by ``position_scale`` (Prior 1.0 = no-op).
        """
        spec = self._protocol.get_position_parse() if self._protocol else None
        try:
            if spec is None or spec.get("format", "csv") == "csv":
                # Prior CSV path (protocol-less OR any csv-format protocol).
                text = response.strip() if response is not None else response
                if spec is not None and spec.get("ack_prefix") and text.startswith(spec["ack_prefix"]):
                    text = text[len(spec["ack_prefix"]):].strip()
                order = spec["axis_order"] if spec else ["x", "y", "z"]
                strip_tokens = (spec["strip_tokens"] if spec else ["R"]) or []
                values = text.split(",")
                if len(values) != len(order):
                    raise ValueError(f"Expected {len(order)} values, got {len(values)}: {response}")
                cleaned = []
                for v in values:
                    v = v.strip().replace("\r", "")
                    for tok in strip_tokens:
                        v = v.strip(tok)
                    cleaned.append(float(v))
                scale = self._position_scale()
                by_axis = {ax: (val / scale if scale else val) for ax, val in zip(order, cleaned)}
                return (by_axis.get("x"), by_axis.get("y"),
                        by_axis.get("z") if by_axis.get("z") is not None else 0.0)

            # Non-CSV (LEP MAC 5000): strip ack prefix, split, map axes, rescale.
            text = response.strip()
            prefix = spec.get("ack_prefix")
            if prefix and text.startswith(prefix):
                text = text[len(prefix):].strip()
            order = spec["axis_order"]
            fmt = spec.get("format", "whitespace")
            if fmt == "regex" and spec.get("regex"):
                m = re.match(spec["regex"], text)
                if not m:
                    raise ValueError(f"Position regex no match: {response}")
                gd = m.groupdict()
                axis_vals = [float(gd[ax]) if gd.get(ax) not in (None, "") else None for ax in order]
            else:  # whitespace
                strip_tokens = spec.get("strip_tokens") or []
                parts = [p for p in text.split() if p != ""]
                cleaned = []
                for p in parts:
                    p = p.replace("\r", "")
                    for tok in strip_tokens:
                        p = p.strip(tok)
                    cleaned.append(p)
                axis_vals = [float(cleaned[i]) if i < len(cleaned) and cleaned[i] not in ("", None) else None
                             for i in range(len(order))]
            scale = self._position_scale()
            by_axis = {ax: (val / scale if (val is not None and scale) else val)
                       for ax, val in zip(order, axis_vals)}
            z = by_axis.get("z")
            return (by_axis.get("x"), by_axis.get("y"), z if z is not None else 0.0)
        except (ValueError, AttributeError, TypeError, KeyError) as e:
            logger.debug(f"Failed to parse XY position: {e}")
            return (None, None, None)

    # ── Movement Commands (P8.19) ─────────────────────────────────

    def move_stage_at_velocity(self, vx: float, vy: float) -> None:
        """Set XY velocity (continuous jog mode).
        v7.3.4: consume 'R' ack under serial lock — proscan_ii.json confirms
        VS responds with R, same as GR/G. Without this, every velocity command
        leaves an unread R\\r in the RX buffer; PositionPoller then reads those
        accumulated R bytes instead of actual position responses, causing
        progressively worsening position display lag during Xbox jogging.

        v7.5.x: controllers with NO continuous-velocity command at all (the
        LEP MAC 5000 HLC command set — ``set_velocity`` is ``null`` in
        mac5000.json, since MOVE/MOVREL are the only motion primitives) fall
        back to a PULSED relative-move jog (see ``_jog_pulse``) so the Xbox
        stick still moves the stage instead of being a silent no-op. Prior
        (``set_velocity`` supported) is completely unaffected.
        """
        if self._protocol is not None and not self._protocol.has_command("set_velocity"):
            self._jog_pulse(vx, vy)
            return
        if self.simulate:
            self._send_protocol_command(
                "set_velocity",
                fallback_cmd=f"VS,{vx},{vy}",
                vx=vx, vy=vy,
            )
        elif self.spo is not None:
            if self._protocol:
                cmd = self._protocol.format_command("set_velocity", vx=vx, vy=vy)
            else:
                cmd = f"VS,{vx},{vy}"
            if cmd:
                if self._protocol:
                    encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{cmd}\r\n".encode("ascii")
                try:
                    with self._serial_lock:
                        self.spo.write(encoded)
                        self._drain_ack(timeout=0.05)
                except Exception as e:
                    logger.debug(f"XY move_stage_at_velocity: {e}")
        _dbg.log("MOVE_VEL", vx=f"{vx:.1f}", vy=f"{vy:.1f}")

    def _jog_pulse(self, vx: float, vy: float) -> None:
        """Emulate continuous-velocity jog via periodic relative moves.

        Controllers whose ASCII command set has no continuous-velocity
        primitive (the LEP MAC 5000 HLC — MOVE/MOVREL only, no VS-equivalent)
        can't be driven the way Prior's VS works. Xbox-stick jog still calls
        ``move_stage_at_velocity`` at a steady ~0.1 s cadence
        (``XYJogHandler._jog_loop``'s ``update_interval``) regardless of
        controller family, so this integrates ``(vx, vy)`` — µm/s — over the
        WALL-CLOCK time actually elapsed since the last pulse (robust to
        scheduler jitter) into a running accumulator, and fires a real
        ``move_stage_relative`` once the accumulated delta clears one wire
        count. That threshold avoids flooding the serial link / controller
        with sub-resolution MOVRELs at a low jog % or a coarse µm-per-count
        scale (each would round to 0 counts and do nothing anyway).

        ``vx == vy == 0`` (stick released, or the jog-handler staleness
        watchdog zeroing) resets the accumulator/timer so the next jog start
        doesn't inherit a stale elapsed-time gap and lurch.
        """
        if vx == 0.0 and vy == 0.0:
            self._jog_pulse_last_t = None
            self._jog_pulse_accum_x = 0.0
            self._jog_pulse_accum_y = 0.0
            return
        now = time.monotonic()
        last = getattr(self, "_jog_pulse_last_t", None)
        self._jog_pulse_last_t = now
        if last is None:
            # First pulse after a stop/start — nothing to integrate yet.
            return
        dt = max(0.0, min(now - last, 0.5))  # cap a stall/GC-pause from over-integrating
        ax = getattr(self, "_jog_pulse_accum_x", 0.0) + vx * dt
        ay = getattr(self, "_jog_pulse_accum_y", 0.0) + vy * dt
        scale = self._position_scale()
        min_um = max(0.5, 1.0 / scale) if scale else 0.5  # at least one wire count
        if abs(ax) < min_um and abs(ay) < min_um:
            self._jog_pulse_accum_x, self._jog_pulse_accum_y = ax, ay
            return
        self._jog_pulse_accum_x = self._jog_pulse_accum_y = 0.0
        self.move_stage_relative(ax, ay)

    def move_stage_to_position(self, x: float, y: float, fast: bool = False) -> None:
        """Move to absolute position (x, y) in stage coordinates.

        v7.2.9: Consumes the "R" response under serial lock to prevent
        buffer pollution that breaks PositionPoller queries.
        """
        # v7.5.x: µm → wire units (Prior scale 1.0 ⇒ round(x*1.0)==round(x)).
        scale = self._position_scale()
        xw, yw = round(x * scale), round(y * scale)
        if self.simulate:
            self._send_protocol_command(
                "move_absolute",
                fallback_cmd=f"G {xw},{yw}",
                x=xw, y=yw,
            )
        elif self.spo is not None:
            if self._protocol:
                cmd = self._protocol.format_command(
                    "move_absolute", x=xw, y=yw)
            else:
                cmd = f"G {xw},{yw}"
            if cmd:
                if self._protocol:
                    encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{cmd}\r\n".encode("ascii")
                try:
                    with self._serial_lock:
                        self.spo.write(encoded)
                        self._drain_ack(timeout=0.05)
                except Exception as e:
                    logger.debug(f"XY move_stage_to_position: {e}")
        logger.debug(f"XY absolute move: ({x:.0f}, {y:.0f}) fast={fast}")

    def move_stage_relative(self, dx: float, dy: float) -> None:
        """Move by relative offset (dx, dy).
        v7.3.4: consume 'R' ack under serial lock — same as move_stage_to_position.
        Without this the ack lingers in the serial buffer and the PositionPoller
        reads 'R' instead of the actual position on its next query.
        """
        # v7.5.x: µm → wire units (Prior scale 1.0 ⇒ no change).
        scale = self._position_scale()
        dxw, dyw = round(dx * scale), round(dy * scale)
        if self.simulate:
            self._send_protocol_command(
                "move_relative",
                fallback_cmd=f"GR {dxw},{dyw}",
                dx=dxw, dy=dyw,
            )
        elif self.spo is not None:
            if self._protocol:
                cmd = self._protocol.format_command(
                    "move_relative", dx=dxw, dy=dyw)
            else:
                cmd = f"GR {dxw},{dyw}"
            if cmd:
                if self._protocol:
                    encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{cmd}\r\n".encode("ascii")
                try:
                    with self._serial_lock:
                        self.spo.write(encoded)
                        self._drain_ack(timeout=0.05)
                except Exception as e:
                    logger.debug(f"XY move_stage_relative: {e}")
        _dbg.log("MOVE_REL", sent_dx=f"{dx:.1f}", sent_dy=f"{dy:.1f}")
        logger.debug(f"XY relative move: ({dx:.0f}, {dy:.0f})")

    def set_home(self) -> None:
        """Set current position as home (0, 0, 0).
        v7.5.0: consume the 'R' ack under serial lock — same as the movement
        commands. Without this the ack lingers in the RX buffer and the
        PositionPoller reads 'R' instead of the zeroed position on its next
        query, so the display slowly counts down to 0 instead of snapping.
        """
        if self.simulate:
            self._send_protocol_command("set_home", fallback_cmd="Z")
        elif self.spo is not None:
            if self._protocol:
                cmd = self._protocol.format_command("set_home")
            else:
                cmd = "Z"
            if cmd:
                if self._protocol:
                    encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{cmd}\r\n".encode("ascii")
                try:
                    with self._serial_lock:
                        self.spo.write(encoded)
                        self._drain_ack(timeout=0.05)
                except Exception as e:
                    logger.debug(f"XY set_home: {e}")
        logger.info("XY home position set")

    def stop_stage(self) -> None:
        """Send immediate stop command.
        v7.5.0: consume the 'R' ack under serial lock — same rationale as
        set_home; an orphaned ack pollutes the next PositionPoller read.
        """
        if self.simulate:
            self._send_protocol_command("stop", fallback_cmd="I")
        elif self.spo is not None:
            if self._protocol:
                cmd = self._protocol.format_command("stop")
            else:
                cmd = "I"
            if cmd:
                if self._protocol:
                    encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
                else:
                    encoded = f"{cmd}\r\n".encode("ascii")
                try:
                    with self._serial_lock:
                        self.spo.write(encoded)
                        self._drain_ack(timeout=0.05)
                except Exception as e:
                    logger.debug(f"XY stop_stage: {e}")
        logger.info("XY stage stopped")

    # ── Stage Settings (P8.19, P8.20) ────────────────────────────

    def _max_speed_um_s(self) -> float:
        """The stage's TRUE top speed (µm/s) at 100% SMS — the denominator for
        the mm/s ↔ SMS-% conversion.

        v7.5.x BUGFIX: this used to read ``_protocol_max_speed_um_s``, which is
        set ONLY in the simulator branch — so on real hardware the conversion
        silently fell back to a hardcoded 50000 µm/s (XYStage.py), making e.g.
        ``set_speed_mm_s(1.0)`` send ``SMS,2`` against a 50 mm/s assumption that
        was never sourced from the real stage. Now resolve, in priority order:
          1. a measured/configured per-machine override (``set_max_speed_um_s``),
          2. the sim override (``_protocol_max_speed_um_s``, sim only),
          3. the loaded protocol's ``parameters.max_speed``,
          4. a 50000 µm/s last resort (logged once).
        The conversion is only CORRECT once (1) or (3) reflects the real stage —
        confirm/measure it; 50000 is an unverified guess.
        """
        for attr in ("_max_speed_override_um_s", "_protocol_max_speed_um_s"):
            v = getattr(self, attr, None)
            if v:
                return float(v)
        try:
            ms = self._protocol._config.get("parameters", {}).get("max_speed")
            if ms:
                return float(ms)
        except Exception:
            pass
        if not getattr(self, "_warned_default_max_speed", False):
            self._warned_default_max_speed = True
            logger.warning(
                "XY top speed unknown (no per-machine override and no protocol "
                "'max_speed') — mm/s↔SMS%% conversion uses the 50000 µm/s "
                "default, which is an UNVERIFIED guess. Measure/set the real "
                "stage top speed or moves will run at the wrong speed.")
        return 50000.0

    def set_max_speed_um_s(self, value_um_s: float) -> None:
        """Set the measured/known true top speed (µm/s at 100% SMS) for this
        machine, so the mm/s↔SMS-% conversion is correct. Persisted by the
        caller (e.g. the device profile)."""
        self._max_speed_override_um_s = float(value_um_s)
        logger.info("XY top speed set to %.0f µm/s (%.1f mm/s) — mm/s↔SMS%% "
                    "conversion now uses this.", value_um_s, value_um_s / 1000.0)

    def _speed_model(self) -> str:
        """'percentage' (Prior SMS) or 'absolute' (LEP MAC 5000 SPEED)."""
        return self._protocol.speed_model if self._protocol else "percentage"

    def _set_speed_absolute(self, speed_mm_s: float) -> None:
        """Absolute-speed model (e.g. Ludl ``SPEED``).

        mm/s → µm/s (clamped to the stage µm/s ceiling from ``_max_speed_um_s``)
        → wire units (× ``position_scale`` when the protocol's ``speed_units`` is
        count/pulse based; else µm/s) → clamped to the protocol ``speed_range``.
        Formats per-axis (``sx``/``sy``) when ``speed_is_per_axis``, else
        ``speed``. Clamping to the stage max is a SAFETY guard so a mis-tuned
        speed can't command an unsafe fast-travel velocity.
        """
        speed_um_s = max(0.0, speed_mm_s * 1000.0)
        ceiling = self._max_speed_um_s()
        if ceiling:
            speed_um_s = min(speed_um_s, ceiling)
        units = (self._protocol.speed_units or "").lower() if self._protocol else ""
        if units in ("counts_per_s", "counts", "microsteps_per_s", "pulses_per_s"):
            wire = speed_um_s * self._position_scale()
        else:  # "um_per_s" or unknown → send µm/s directly
            wire = speed_um_s
        rng = self._protocol.get_speed_range() if self._protocol else None
        if rng:
            wire = max(rng[0], min(rng[1], wire))
        wire = int(round(wire))
        logger.info("set_speed(absolute): %.2f mm/s → %d wire (units=%s, scale=%.3f)",
                    speed_mm_s, wire, units or "um_per_s", self._position_scale())
        if self._protocol and self._protocol.speed_is_per_axis:
            self._send_protocol_command("set_max_speed", sx=wire, sy=wire)
        else:
            self._send_protocol_command("set_max_speed", fallback_cmd=f"SMS,{wire}", speed=wire)

    def set_velocity(self, velocity: int) -> None:
        """v7.2.7: Set max stage velocity.

        Percentage model (Prior SMS): the value is a percentage (1-100); a value
        > 100 is treated as µm/s and converted. Absolute model (Ludl): a value
        <= 100 is a percentage of the stage's µm/s ceiling; > 100 is µm/s.

        For explicit mm/s control, use set_speed_mm_s() instead.
        """
        if self._speed_model() == "absolute":
            if velocity <= 100:
                frac = max(1, min(100, int(velocity))) / 100.0
                self._set_speed_absolute((self._max_speed_um_s() * frac) / 1000.0)
            else:
                self._set_speed_absolute(velocity / 1000.0)  # µm/s → mm/s
            return
        if velocity > 100:
            # Caller sent µm/s — convert to percentage
            max_speed = self._max_speed_um_s()
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
        """v7.2.7: Set stage speed in mm/s.

        This is the preferred method for all print/calibration code. Percentage
        model (Prior): mm/s → µm/s → percentage of max → ``SMS``. Absolute model
        (Ludl): mm/s → wire units → ``SPEED`` (see ``_set_speed_absolute``).

        Args:
            speed_mm_s: Desired speed in mm/s (e.g., 1.0, 5.0, 50.0)
        """
        if self._speed_model() == "absolute":
            self._set_speed_absolute(speed_mm_s)
            return
        max_speed_um_s = self._max_speed_um_s()
        speed_um_s = speed_mm_s * 1000.0
        pct = max(1, min(100, int(round(speed_um_s / max_speed_um_s * 100))))
        logger.info(f"set_speed_mm_s: {speed_mm_s:.1f} mm/s = {speed_um_s:.0f} µm/s "
                    f"= SMS {pct}% (max={max_speed_um_s:.0f} µm/s)")
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

    def _query_protocol_value(
        self, command_name: str, axis: str = "X", timeout: float = 0.5
    ) -> "Optional[str]":
        """Send a QUERY command and return its raw response text, or None.

        Unlike ``_send_protocol_command`` (fire-and-forget on real hardware),
        this WRITES the formatted command AND READS the reply — for read-back
        commands (``get_max_speed``/``get_acceleration``) where the caller
        needs the returned value, not just an ack. Best-effort: returns None
        if the command is unsupported by the loaded protocol, the stage isn't
        connected, or on any I/O error (never raises).
        """
        if self._protocol is None:
            return None
        cmd = self._protocol.format_command(command_name, axis=axis)
        if cmd is None:
            return None  # not supported by this protocol
        if self.simulate:
            if self.spo is None:
                return None
            try:
                return self.spo.send_command(cmd)
            except Exception as e:
                logger.debug(f"XY query '{command_name}' (sim) failed: {e}")
                return None
        if self.spo is None:
            return None
        try:
            encoded = cmd.encode(self._protocol.encoding) + self._protocol.tx_terminator
            with self._serial_lock:
                self.spo.write(encoded)
                return _read_response_cr(self.spo, timeout=timeout)
        except Exception as e:
            logger.debug(f"XY query '{command_name}' failed: {e}")
            return None

    @staticmethod
    def _extract_first_int(text: "Optional[str]") -> "Optional[int]":
        """Parse the first integer in a reply.

        Tolerates a leading ``:A``/``:N`` ack prefix and any trailing junk —
        some Ludl single-axis ``?`` query replies append an unrelated token
        for the axis that WASN'T asked about (e.g. ``ACCEL X?`` → ``:A 20
        N-2`` — confirmed on real hardware). An explicit ``:N`` error reply
        returns None rather than parsing whatever number follows the dash.
        """
        if not text:
            return None
        text = text.strip()
        if text.startswith(":N"):
            return None
        m = re.search(r"-?\d+", text)
        return int(m.group(0)) if m else None

    def get_speed_readback(self, axis: str = "X") -> dict:
        """Query the stage's CURRENT max-speed setting directly from hardware.

        v7.5.x: added after a real MAC 5000 incident where a malformed
        command silently corrupted ONE axis's SPEED register while the other
        was untouched — there was no way to SEE that divergence from the app.
        Returns ``{'raw', 'axis', 'model', 'display'}``; best-effort —
        ``raw=None`` / ``display='unavailable'`` on any failure (unsupported
        command, error/malformed reply, not connected). Never raises.
        """
        value = self._extract_first_int(
            self._query_protocol_value("get_max_speed", axis=axis))
        result = {"raw": value, "axis": axis, "model": self._speed_model(),
                  "display": "unavailable"}
        if value is None:
            return result
        if self._speed_model() == "absolute":
            scale = self._position_scale()
            um_s = value / scale if scale else float(value)
            result["display"] = f"{value} ({um_s:.0f} µm/s / {um_s / 1000.0:.2f} mm/s)"
        else:
            result["display"] = f"{value}%"
        return result

    def get_acceleration_readback(self, axis: str = "X") -> dict:
        """Query the stage's CURRENT acceleration setting directly from
        hardware. See ``get_speed_readback`` for the motivating incident."""
        value = self._extract_first_int(
            self._query_protocol_value("get_acceleration", axis=axis))
        accel_model = self._protocol.accel_model if self._protocol else "percentage"
        result = {"raw": value, "axis": axis, "model": accel_model,
                  "display": "unavailable"}
        if value is None:
            return result
        if accel_model == "absolute":
            result["display"] = f"{value} (ramp index, LOWER = faster)"
        else:
            result["display"] = f"{value}%"
        return result

    def set_fast_mode(self) -> None:
        """Set velocity to maximum."""
        self.set_velocity(self.max_speed)

    def set_slow_mode(self) -> None:
        """Set velocity to default."""
        self.set_velocity(self.default_velocity)

    def get_firmware_version(self) -> Optional[str]:
        """Query controller firmware version.
        v7.2.8s2: atomic with serial lock + CR-aware read.
        """
        if self.simulate:
            return "Simulator v1.0"
        try:
            with self._serial_lock:
                self._send_protocol_command("firmware_version", fallback_cmd="V")
                response = _read_response_cr(self.spo, timeout=0.5)
            return response if response else None
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
