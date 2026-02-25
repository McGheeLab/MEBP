"""
Settings — Persistent application configuration via JSON.

Supports dot-path access (e.g. ``settings.get("window.width")``),
deep-merge on load (new defaults are preserved), and section-level
get/set for bulk operations.

Usage::

    settings = Settings()
    settings.load()
    width = settings.get("window.width")
    settings.set("speeds.xy", 500.0)
    settings.save()
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger(__name__)

DEFAULT_SETTINGS_FILE = "settings.json"

DEFAULTS: dict[str, Any] = {
    "window": {
        "x": 100, "y": 100,
        "width": 1200, "height": 800,
        "active_tab": 0,
        "splitter_sizes": [500, 150],
    },
    "simulation": {
        "simulate_xy": True,
        "simulate_zp": True,
    },
    "speeds": {
        "xy": 250.0,
        "z": 1.0,
        "p": 0.5,
    },
    "zero_position": {
        "x": 0.0, "y": 0.0, "f": 0.0,
        "Z": 0.0, "P1": 0.0, "P2": 0.0, "P3": 0.0,
    },
    "print_settings": {
        "travel_z_height": 5.0,
        "print_z_height": 0.1,
        "layer_height": 0.1,
        "num_layers": 1,
        "print_feedrate": 200.0,
        "z_feedrate": 60.0,
        "pump_feedrate": 30.0,
        "flow_rate": 0.01,
        "retract_amount": 0.0,
        "prime_amount": 0.0,
        "dwell_after_move": 0.0,
    },
    "xbox": {
        "mapping_file": "current_button_mapping.json",
    },
    "logging": {
        "verbose": False,
    },
    "safety_limits": {
        "xy_min_x": -100_000.0,
        "xy_min_y": -100_000.0,
        "xy_max_x": 100_000.0,
        "xy_max_y": 100_000.0,
        "z_min": -10.0,
        "z_max": 50.0,
        "p1_min": -50.0,
        "p1_max": 50.0,
        "p2_min": -50.0,
        "p2_max": 50.0,
        "p3_min": -50.0,
        "p3_max": 50.0,
        "max_xy_speed": 10_000.0,
        "max_z_feedrate": 500.0,
        "max_pump_feedrate": 200.0,
        "enabled": True,
    },
    "polling": {
        "position_interval_ms": 300,
        "watchdog_interval_s": 3.0,
    },
    "calibration": {
        "plate_format": 96,
        "taught_a1": None,
        "taught_corner": None,
        "corner_well": "H12",
        "offset_x": 0.0,
        "offset_y": 0.0,
        "rotation": 0.0,
        "scale": 1.0,
        "alignment_valid": False,
    },
}


class Settings:
    """
    JSON-backed application settings with dot-path access.

    Missing keys fall back to :data:`DEFAULTS` on load via deep merge.
    """

    def __init__(self, filepath: str = DEFAULT_SETTINGS_FILE):
        self.filepath = Path(filepath)
        self._data: dict = self._deep_copy(DEFAULTS)

    # ── Load / Save ───────────────────────────────────────────────

    def load(self) -> None:
        """Load from file, falling back to defaults for missing keys."""
        if not self.filepath.exists():
            logger.info(f"No settings file at {self.filepath} — using defaults")
            return
        try:
            with open(self.filepath) as f:
                saved = json.load(f)
            self._merge(self._data, saved)
            logger.info(f"Settings loaded from {self.filepath}")
        except json.JSONDecodeError as e:
            logger.warning(f"Corrupt settings file ({e}) — using defaults")
        except Exception as e:
            logger.warning(f"Failed to load settings: {e} — using defaults")

    def save(self) -> None:
        """Persist current settings to file."""
        try:
            with open(self.filepath, "w") as f:
                json.dump(self._data, f, indent=2)
            logger.debug(f"Settings saved to {self.filepath}")
        except Exception as e:
            logger.error(f"Failed to save settings: {e}")

    # ── Dot-Path Access ───────────────────────────────────────────

    def get(self, dotpath: str, default: Any = None) -> Any:
        """
        Read a value by dot-separated path.

        Example: ``settings.get("window.width")`` → ``1200``
        """
        node = self._data
        for key in dotpath.split("."):
            if isinstance(node, dict) and key in node:
                node = node[key]
            else:
                return default
        return node

    def set(self, dotpath: str, value: Any) -> None:
        """
        Write a value by dot-separated path, creating intermediate dicts.

        Example: ``settings.set("speeds.xy", 500.0)``
        """
        keys = dotpath.split(".")
        node = self._data
        for key in keys[:-1]:
            if key not in node or not isinstance(node[key], dict):
                node[key] = {}
            node = node[key]
        node[keys[-1]] = value

    # ── Section Access ────────────────────────────────────────────

    def get_section(self, section: str) -> dict:
        """Return a deep copy of an entire top-level section."""
        return self._deep_copy(self._data.get(section, {}))

    def set_section(self, section: str, data: dict) -> None:
        """Replace an entire top-level section."""
        self._data[section] = self._deep_copy(data)

    # ── Utility ───────────────────────────────────────────────────

    @property
    def data(self) -> dict:
        """Full settings dict (deep copy — safe to mutate)."""
        return self._deep_copy(self._data)

    @staticmethod
    def _deep_copy(d: dict) -> dict:
        """Fast deep copy for JSON-serialisable dicts."""
        return json.loads(json.dumps(d))

    @staticmethod
    def _merge(base: dict, override: dict) -> None:
        """Recursively merge *override* into *base* (in-place)."""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                Settings._merge(base[key], value)
            else:
                base[key] = value

    def __repr__(self) -> str:
        return f"Settings(file={self.filepath})"
