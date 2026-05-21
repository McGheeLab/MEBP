"""
Settings — Persistent application configuration via JSON.

Supports dot-path access (e.g. ``settings.get("window.width")``),
deep-merge on load (new defaults are preserved), and section-level
get/set for bulk operations.

v7.1 additions (P8.30–P8.33):
- controller: JSON protocol path + auto-detect result
- workspace: needle, pump, plate config persistence
- ink_library: user-defined inks
- rosette_library: user-defined rosette inserts
- well_setup: well assignment persistence
- motion_controller: PID/Kalman tuning + rate test results
- fluid_columns: per-pump fluid column state persistence

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
        "simulate_xy": False,  # v7.2.8s2: default simulate False (lab instrument)
        "simulate_zp": False,
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
        "xy_min_x": -130_000.0,
        "xy_min_y": -85_000.0,
        "xy_max_x": 130_000.0,
        "xy_max_y": 85_000.0,
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
    # ── v7.1 additions (P8.30–P8.33) ─────────────────────────────
    "controller": {
        # P8.33: Controller protocol JSON path
        # "auto" = auto-detect, null = default ProScan III, or explicit path
        "controller_json": None,
        "auto_detect_result": None,         # Name of last auto-detected controller
        "controllers_dir": "config/controllers",
    },
    "workspace": {
        # P8.30: Workspace configuration persistence
        "needle_gauge": None,               # Last selected needle gauge (int)
        "plate_format": 24,                 # Last selected plate format
        "pump_loadouts": {                  # Per-pump syringe and ink assignments
            "P1": {"syringe_volume_uL": None, "ink_name": None, "printing_mode": "incremental"},
            "P2": {"syringe_volume_uL": None, "ink_name": None, "printing_mode": "incremental"},
            "P3": {"syringe_volume_uL": None, "ink_name": None, "printing_mode": "incremental"},
        },
        "buffer_ink_name": None,
    },
    "ink_library": {
        # P8.30: User-defined inks (name → InkSpec dict)
        "inks": {},
    },
    "rosette_library": {
        # P8.30: User-defined rosette inserts (name → RosetteInsert dict)
        "rosettes": {},
    },
    "well_setup": {
        # P8.30: Well setup persistence
        "last_saved_file": None,            # Path to last saved well setup JSON
        "auto_save": True,                  # Auto-save well setup on changes
    },
    "motion_controller": {
        # P8.30/P8.31: Motion controller tuning and rate test results
        "controller_type": "pid",           # "pid" or "kalman"
        "pid_kp": 1.0,
        "pid_ki": 0.0,
        "pid_kd": 0.1,
        "kalman_process_noise": 0.01,
        "kalman_measurement_noise": 0.1,
        "rate_test_results": None,          # P8.31: Last test_command_rate() results
    },
    "fluid_columns": {
        # P8.32: Fluid column state persistence (save/restore between sessions)
        # Per-pump fluid column state (oil_uL, buffer_uL, ink_uL, ink_name)
        "P1": None,
        "P2": None,
        "P3": None,
    },
    # v7.2: Hardware configuration persistence
    "hardware_config": {
        "last_config_file": None,           # Path to last loaded hardware config JSON
        "auto_load": True,                  # Auto-load last config on startup
    },
    # v7.4.0-b: Migration tracking. Each one-shot migration sets a flag here
    # so it only runs once per settings.json file. See _run_migrations().
    "migrations": {
        "v7_4_0_b": False,
        "v7_4_1_default_device": False,
    },
    # v7.4.1: Device profile (initial device setup — safety, feedrates,
    # axis flips). The active profile name is tracked here so we can
    # restore the selection in the Stage sub-page UI on each launch.
    #
    # v7.4.2: axis_map and steps_per_mm added. axis_map maps logical
    # axes (Z, P1, P2, P3) to physical Marlin axes (X, Y, Z, E).
    # steps_per_mm is per-LOGICAL-axis (sign indicates direction).
    "device_profile": {
        "active": None,
        "axis_map": {
            "Z": "X",   # vertical needle  → Marlin X
            "P1": "Y",  # syringe pump 1   → Marlin Y
            "P2": "Z",  # syringe pump 2   → Marlin Z
            "P3": "E",  # syringe pump 3   → Marlin E
        },
        "steps_per_mm": {
            "Z": 5069,
            "P1": 5069,
            "P2": -5069,   # negative = inverted direction (legacy default)
            "P3": 5069,
        },
        # v7.4.2: per-axis max feedrate (mm/min) discovered via the
        # Stepper Calibration feedrate-test workflow. Recorded here
        # for reference; the user later copies values into the global
        # safety_limits.max_z_feedrate / max_pump_feedrate as desired.
        "per_axis_max_feedrate": {
            "Z": 500,
            "P1": 200,
            "P2": 200,
            "P3": 200,
        },
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
        """Load from file, falling back to defaults for missing keys.

        v7.4.0-b: After loading, runs one-shot migrations (see
        :meth:`_run_migrations`). The first such migration writes a
        ``settings.json.bak-v7.3`` backup so users can roll back if
        anything goes wrong.
        """
        if not self.filepath.exists():
            logger.info(f"No settings file at {self.filepath} — using defaults")
            return
        try:
            with open(self.filepath) as f:
                saved = json.load(f)
            self._merge(self._data, saved)
            logger.info(f"Settings loaded from {self.filepath}")
            self._run_migrations()
        except json.JSONDecodeError as e:
            logger.warning(f"Corrupt settings file ({e}) — using defaults")
        except Exception as e:
            logger.warning(f"Failed to load settings: {e} — using defaults")

    # ── Migrations (v7.4.0-b) ─────────────────────────────────────

    def _run_migrations(self) -> None:
        """Run one-shot data migrations idempotently.

        Each migration:
          * checks its flag under ``migrations.<name>``
          * if unset, performs its work and sets the flag
          * the next ``save()`` persists the flag so it never re-runs

        v7.4.0-b: Writes a single ``.bak-v7.3`` snapshot of the loaded
        settings before the new layout is allowed to overwrite it.
        Future v7.4.0-c migrations (e.g., moving keys into the
        hardware_config namespace) can be added here.
        """
        if not self.get("migrations.v7_4_0_b", False):
            self._backup_v73()
            self.set("migrations.v7_4_0_b", True)
            self.save()
            logger.info("v7.4.0-b migration complete; .bak-v7.3 written")

        # v7.4.1: If no device profile has been selected yet, auto-load
        # the bundled Standard.json so users have a sensible safety
        # envelope from the very first launch.
        if not self.get("migrations.v7_4_1_default_device", False):
            self._apply_default_device_profile()
            self.set("migrations.v7_4_1_default_device", True)
            self.save()
            logger.info("v7.4.1 default device profile applied")

    def _apply_default_device_profile(self) -> None:
        """v7.4.1: Apply Standard.json device profile on first launch.

        Only runs if no active profile is currently set — won't clobber
        a user who already picked a profile manually.
        """
        if self.get("device_profile.active"):
            return  # Already configured
        try:
            # Lazy import to avoid GUI dependency in headless contexts
            from pathlib import Path
            standard_path = (
                Path(__file__).resolve().parent.parent
                / "config" / "hardware" / "devices" / "Standard.json"
            )
            if not standard_path.exists():
                logger.debug(
                    "Standard device profile not found; skipping default")
                return
            data = json.loads(standard_path.read_text())
            for section in ("safety_limits", "zp_stage", "axis_flip"):
                if section in data and isinstance(data[section], dict):
                    self.set_section(section, data[section])
            self.set("device_profile.active",
                     data.get("profile_name", "Standard"))
            logger.info("Applied default device profile: Standard")
        except Exception as e:
            logger.warning(f"Failed to apply default device profile: {e}")

    def _backup_v73(self) -> None:
        """Write a one-time backup of the current settings.json file.

        Skips if the backup already exists.
        """
        backup = self.filepath.with_suffix(self.filepath.suffix + ".bak-v7.3")
        if backup.exists():
            logger.debug(f"v7.3 backup already present at {backup} — skipping")
            return
        try:
            with open(self.filepath) as src, open(backup, "w") as dst:
                dst.write(src.read())
            logger.info(f"v7.3 settings backup written to {backup}")
        except Exception as e:
            logger.warning(f"Failed to write v7.3 backup: {e}")

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
