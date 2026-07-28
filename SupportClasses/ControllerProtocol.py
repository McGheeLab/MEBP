"""
ControllerProtocol.py — JSON-based controller protocol mapping for MEBP v7.1.

Loads controller-specific command maps from JSON files in config/controllers/.
Provides command formatting, auto-detection, and protocol abstraction so that
adding a new XY controller type is simply adding a new JSON file.

Supported controllers:
- Prior ProScan II  (config/controllers/proscan_ii.json)
- Prior ProScan III (config/controllers/proscan_iii.json)
- Future: ASI MS-2000, Zaber, etc. — just add JSON files

Usage:
    proto = ControllerProtocol.load("config/controllers/proscan_iii.json")
    cmd = proto.format_command("move_absolute", x=1000, y=2000)
    # → "G 1000,2000"
    terminator = proto.tx_terminator  # → b"\\r\\n"
"""

from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Default directory for controller JSON files
DEFAULT_CONTROLLERS_DIR = "config/controllers"


class ControllerProtocol:
    """
    Loads a controller JSON map and provides command formatting.

    The JSON file defines:
    - Communication settings (baud rate, terminators, encoding)
    - Detection sequence (for auto-detect)
    - Command map (abstract name → controller-specific syntax)
    - Parameter ranges (speed, acceleration, position limits)
    """

    def __init__(self, config: dict, source_path: str = ""):
        self._config = config
        self._source_path = source_path
        self._commands = config.get("commands", {})

        # Parse communication settings
        comm = config.get("communication", {})
        self._tx_term = self._parse_terminator(comm.get("line_terminator_tx", "\\r\\n"))
        self._rx_term = self._parse_terminator(comm.get("line_terminator_rx", "\\r\\n"))
        self._baud_rate = comm.get("default_baud_rate", 38400)
        self._byte_size = comm.get("byte_size", 8)
        self._stop_bits = comm.get("stop_bits", 1)
        self._timeout = comm.get("timeout_s", 1.0)
        self._encoding = comm.get("encoding", "ascii")

        # --- v7.5.x: extended semantics for non-Prior XY controllers (LEP MAC 5000) ---
        # Every field below is OPTIONAL; when absent the getter returns the value that
        # reproduces the historic hardcoded Prior behavior, so proscan_ii.json /
        # proscan_iii.json (which set none of them) stay byte-for-byte identical.
        self._family = str(config.get("controller_family", "prior")).lower()

        # Raw command-mode init bytes (e.g. Ludl "FF41" = 0xFF 0x41 forces ASCII /
        # high-level mode). Written verbatim at connect with NO line terminator.
        self._command_mode_init_bytes = self._parse_init_bytes(
            config.get("command_mode_init_hex"))

        # Ack / error semantics. Prior writes reply with a bare "R"; Ludl replies ":A"
        # on success and ":N -<code>" on error.
        rs = config.get("response_semantics", {}) or {}
        self._ack_success_token = rs.get("ack_success_token", "R")
        err_pat = rs.get("ack_error_pattern")
        try:
            self._ack_error_re = re.compile(err_pat) if err_pat else None
        except re.error:
            logger.error(f"Invalid ack_error_pattern: {err_pat!r}")
            self._ack_error_re = None

        # Position-response parsing. Prior default = comma CSV, strip a trailing "R",
        # axes x,y,z. Ludl WHERE → ":A <x> <y>" (whitespace, 2 axes, ":A" prefix).
        pp = config.get("position_parse", {}) or {}
        self._position_parse = {
            "format": pp.get("format", "csv"),
            "axis_order": list(pp.get("axis_order", ["x", "y", "z"])),
            "strip_tokens": list(pp.get("strip_tokens", ["R"])),
            "ack_prefix": pp.get("ack_prefix"),
            "regex": pp.get("regex"),
        }

        # Speed / acceleration models. Prior = percentage of max (SMS/SAS 1-100);
        # Ludl = absolute per-axis (SPEED counts/s, ACCEL 1-255 ramp index).
        self._speed_model = str(config.get("speed_model", "percentage")).lower()
        self._speed_units = config.get("speed_units")
        self._speed_per_axis = bool(config.get("speed_per_axis", False))
        self._accel_model = str(config.get("accel_model", "percentage")).lower()
        self._accel_units = config.get("accel_units")

    @staticmethod
    def _parse_terminator(term_str: str) -> bytes:
        """Parse escaped terminator string from JSON to bytes."""
        # Handle JSON-escaped sequences like "\\r\\n" → b"\r\n"
        return term_str.encode("utf-8").decode("unicode_escape").encode("ascii")

    @staticmethod
    def _parse_init_bytes(hex_str: Any) -> bytes | None:
        """Parse a hex string like "FF41" (or "0xFF 0x41") into raw bytes.

        Returns None when absent/blank/invalid so callers can skip the write.
        """
        if not hex_str:
            return None
        try:
            cleaned = "".join(str(hex_str).split()).replace("0x", "").replace("0X", "")
            return bytes.fromhex(cleaned)
        except ValueError:
            logger.error(f"Invalid command_mode_init_hex: {hex_str!r}")
            return None

    @classmethod
    def load(cls, filepath: str | Path) -> ControllerProtocol:
        """
        Load a controller protocol from a JSON file.

        Args:
            filepath: Path to the controller JSON file

        Returns:
            ControllerProtocol instance

        Raises:
            FileNotFoundError: If the JSON file doesn't exist
            json.JSONDecodeError: If the JSON is malformed
        """
        filepath = Path(filepath)
        if not filepath.exists():
            raise FileNotFoundError(f"Controller protocol file not found: {filepath}")

        with open(filepath) as f:
            config = json.load(f)

        proto = cls(config, source_path=str(filepath))
        logger.info(f"Loaded controller protocol: {proto.controller_name} from {filepath}")
        return proto

    # --- Properties ---

    @property
    def controller_name(self) -> str:
        return self._config.get("controller_name", "Unknown")

    @property
    def manufacturer(self) -> str:
        return self._config.get("manufacturer", "Unknown")

    @property
    def protocol_version(self) -> str:
        return self._config.get("protocol_version", "")

    @property
    def source_path(self) -> str:
        return self._source_path

    @property
    def tx_terminator(self) -> bytes:
        """Transmit line terminator (bytes to append to commands)."""
        return self._tx_term

    @property
    def rx_terminator(self) -> bytes:
        """Receive line terminator (bytes expected at end of responses)."""
        return self._rx_term

    @property
    def baud_rate(self) -> int:
        return self._baud_rate

    @property
    def baud_rate_candidates(self) -> list[int]:
        """Baud rates to try, in order, during detection on each port.

        v7.5.x: added after a real Prior ProScan III unit was found sitting
        at a non-default baud (Prior's own manual documents that a changed
        baud setting silently reverts to 9600 if the port sits idle across
        TWO power cycles — a real, expected failure mode, not a one-off).
        Optional ``communication.baud_rates`` (a list, highest/preferred
        first) enables retrying several rates per port; absent ⇒ the single
        ``default_baud_rate`` (current behavior, byte-identical for every
        protocol that doesn't declare it — Prior II/III, Ludl unaffected).
        """
        comm = self._config.get("communication", {})
        rates = comm.get("baud_rates")
        if rates:
            try:
                return [int(r) for r in rates]
            except (TypeError, ValueError):
                logger.error(f"Invalid communication.baud_rates: {rates!r}")
        return [self._baud_rate]

    @property
    def byte_size(self) -> int:
        return self._byte_size

    @property
    def stop_bits(self) -> int:
        return self._stop_bits

    @property
    def timeout(self) -> float:
        return self._timeout

    @property
    def encoding(self) -> str:
        return self._encoding

    # --- v7.5.x: family + wire semantics (Prior-safe defaults) ---

    @property
    def family(self) -> str:
        """Controller family: 'prior' (default) or 'ludl'.

        Drives the simulator choice and the coarse semantic branches in XYStage.
        """
        return self._family

    @property
    def command_mode_init_bytes(self) -> bytes | None:
        """Raw bytes to write (no terminator) once at connect, or None.

        Used by the LEP MAC 5000 to force ASCII/high-level mode (0xFF 0x41).
        """
        return self._command_mode_init_bytes

    @property
    def ack_success_token(self) -> str:
        """Token that marks a successful ack ('R' for Prior, ':A' for Ludl)."""
        return self._ack_success_token

    def match_ack_error(self, response: str | None) -> "re.Match | None":
        """Return the regex match if `response` is an error ack, else None."""
        if not response or self._ack_error_re is None:
            return None
        return self._ack_error_re.match(response.strip())

    def get_position_parse(self) -> dict:
        """Position-response parse spec (a copy, Prior defaults filled in)."""
        return dict(self._position_parse)

    @property
    def speed_model(self) -> str:
        """'percentage' (Prior SMS 1-100) or 'absolute' (Ludl SPEED counts/s)."""
        return self._speed_model

    @property
    def speed_units(self) -> str | None:
        """Absolute-speed units, e.g. 'counts_per_s' or 'um_per_s' (None for percentage)."""
        return self._speed_units

    @property
    def speed_is_per_axis(self) -> bool:
        """True when the set_max_speed template takes {sx}/{sy} instead of {speed}."""
        return self._speed_per_axis

    @property
    def accel_model(self) -> str:
        """'percentage' (Prior SAS 1-100) or 'absolute' (Ludl ACCEL 1-255)."""
        return self._accel_model

    @property
    def accel_units(self) -> str | None:
        return self._accel_units

    @property
    def position_scale(self) -> float:
        """Stage wire-units per micron.

        Prior = 1.0 (native µm, so multiplying/dividing is a no-op). Ludl = counts
        per µm (≈10). Applied ONLY inside XYStage on the send/read boundary; every
        layer above XYStage stays µm-native.
        """
        try:
            v = self.get_parameter("xy_position_scale")
            return float(v) if v else 1.0
        except (TypeError, ValueError):
            return 1.0

    # --- Command Formatting ---

    def format_command(self, command_name: str, **kwargs) -> str | None:
        """
        Format a command with parameters.

        Looks up the abstract command name in the JSON command map,
        then substitutes any {parameter} placeholders.

        Args:
            command_name: Abstract command name (e.g. "move_absolute")
            **kwargs: Parameter values (e.g. x=1000, y=2000)

        Returns:
            Formatted command string, or None if command not supported
        """
        entry = self._commands.get(command_name)
        if entry is None:
            return None  # Command not supported by this controller

        try:
            return entry["cmd"].format(**kwargs)
        except KeyError as e:
            logger.error(f"Missing parameter {e} for command '{command_name}'")
            return None

    def has_command(self, command_name: str) -> bool:
        """Check if a command is supported by this controller."""
        return self._commands.get(command_name) is not None

    def get_expected_response(self, command_name: str) -> str | None:
        """Get the expected response pattern for a command."""
        entry = self._commands.get(command_name)
        if entry is None:
            return None
        return entry.get("response")

    def get_all_commands(self) -> list[str]:
        """Get list of all supported command names."""
        return [k for k, v in self._commands.items() if v is not None]

    # --- Detection ---

    def get_detection_info(self) -> dict:
        """Get the auto-detection configuration."""
        return self._config.get("detection", {})

    # --- Parameters ---

    def get_parameter(self, name: str) -> Any:
        """Get a controller parameter value."""
        return self._config.get("parameters", {}).get(name)

    def get_speed_range(self) -> tuple[int, int] | None:
        """Get valid speed range [min, max]."""
        r = self.get_parameter("speed_range")
        return tuple(r) if r else None

    def get_acceleration_range(self) -> tuple[int, int] | None:
        """Get valid acceleration range [min, max]."""
        r = self.get_parameter("acceleration_range")
        return tuple(r) if r else None

    def get_jerk_range(self) -> tuple[int, int] | None:
        """Get valid jerk range [min, max], or None if jerk not supported."""
        r = self.get_parameter("jerk_range")
        return tuple(r) if r else None

    @property
    def supports_jerk(self) -> bool:
        """Whether this controller supports jerk (S-curve) control."""
        return self.has_command("set_jerk") and self.get_jerk_range() is not None

    # --- Serialization ---

    def to_dict(self) -> dict:
        """Return the raw config dict."""
        return dict(self._config)

    def __repr__(self) -> str:
        return f"ControllerProtocol({self.controller_name}, commands={len(self.get_all_commands())})"


# ---------------------------------------------------------------------------
# Auto-Detection
# ---------------------------------------------------------------------------

def discover_controller_files(directory: str | Path = DEFAULT_CONTROLLERS_DIR) -> list[Path]:
    """
    Find all controller JSON files in the given directory.

    Returns:
        Sorted list of JSON file paths
    """
    d = Path(directory)
    if not d.exists():
        logger.warning(f"Controller directory not found: {d}")
        return []
    files = sorted(d.glob("*.json"))
    logger.info(f"Found {len(files)} controller protocol files in {d}")
    return files


def load_all_protocols(directory: str | Path = DEFAULT_CONTROLLERS_DIR) -> dict[str, ControllerProtocol]:
    """
    Load all controller protocols from a directory.

    Returns:
        {controller_name: ControllerProtocol}
    """
    protocols = {}
    for fp in discover_controller_files(directory):
        try:
            proto = ControllerProtocol.load(fp)
            protocols[proto.controller_name] = proto
        except Exception as e:
            logger.warning(f"Failed to load controller protocol {fp}: {e}")
    return protocols
