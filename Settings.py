"""
Settings - Persistent application configuration.

Saves and loads app state from a JSON file:
  - Window geometry (position, size)
  - Simulation mode flags
  - Speed multipliers
  - Zero reference positions
  - Last-used print settings
  - Active tab
  - Xbox mapping file path
"""

import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

DEFAULT_SETTINGS_FILE = "settings.json"

DEFAULTS = {
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
}


class Settings:
    """
    Manages persistent application settings stored as JSON.
    
    Usage:
        settings = Settings()
        settings.load()
        
        # Read/write values with dot path
        width = settings.get("window.width")
        settings.set("speeds.xy", 500.0)
        
        # Save
        settings.save()
    """

    def __init__(self, filepath: str = DEFAULT_SETTINGS_FILE):
        self.filepath = Path(filepath)
        self._data = self._deep_copy(DEFAULTS)

    @staticmethod
    def _deep_copy(d: dict) -> dict:
        """Simple deep copy for nested dicts of primitives."""
        return json.loads(json.dumps(d))

    def load(self):
        """Load settings from file, falling back to defaults for missing keys."""
        if not self.filepath.exists():
            logger.info(f"No settings file found at {self.filepath}, using defaults")
            return

        try:
            with open(self.filepath, "r") as f:
                saved = json.load(f)
            self._merge(self._data, saved)
            logger.info(f"Settings loaded from {self.filepath}")
        except Exception as e:
            logger.warning(f"Failed to load settings: {e}, using defaults")

    def save(self):
        """Save current settings to file."""
        try:
            with open(self.filepath, "w") as f:
                json.dump(self._data, f, indent=2)
            logger.debug(f"Settings saved to {self.filepath}")
        except Exception as e:
            logger.error(f"Failed to save settings: {e}")

    def get(self, dotpath: str, default=None):
        """
        Get a setting value using dot notation.
        
        Example: settings.get("window.width") → 1200
        """
        keys = dotpath.split(".")
        node = self._data
        for key in keys:
            if isinstance(node, dict) and key in node:
                node = node[key]
            else:
                return default
        return node

    def set(self, dotpath: str, value):
        """
        Set a setting value using dot notation.
        
        Example: settings.set("speeds.xy", 500.0)
        """
        keys = dotpath.split(".")
        node = self._data
        for key in keys[:-1]:
            if key not in node or not isinstance(node[key], dict):
                node[key] = {}
            node = node[key]
        node[keys[-1]] = value

    def get_section(self, section: str) -> dict:
        """Get an entire settings section as a dict."""
        return self._deep_copy(self._data.get(section, {}))

    def set_section(self, section: str, data: dict):
        """Set an entire settings section."""
        self._data[section] = self._deep_copy(data)

    @staticmethod
    def _merge(base: dict, override: dict):
        """Recursively merge override into base (base is modified in-place)."""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                Settings._merge(base[key], value)
            else:
                base[key] = value

    @property
    def data(self) -> dict:
        """Full settings dictionary (read-only copy)."""
        return self._deep_copy(self._data)
