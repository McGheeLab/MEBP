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

    @staticmethod
    def _parse_terminator(term_str: str) -> bytes:
        """Parse escaped terminator string from JSON to bytes."""
        # Handle JSON-escaped sequences like "\\r\\n" → b"\r\n"
        return term_str.encode("utf-8").decode("unicode_escape").encode("ascii")

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
