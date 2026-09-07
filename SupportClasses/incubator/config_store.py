"""
config_store.py — per-machine incubator configuration.

Owns everything the operator configures about the incubator on Hardware Setup
→ Incubator: which transport reaches the board, the dedicated-port hint, zone
naming/enable, the setpoint ceiling, ramp/dither preferences, and the per-zone
PID gains.

The PID gains live here — not only in the board's EEPROM — because a Marlin
reset reverts the running gains to whatever EEPROM holds, and this board resets
routinely (opening its serial port pulses DTR, so every ZP auto-reconnect
reboots it). That is the same mechanism that made it forget its *setpoint*
(see ``controller._service_setpoint_keeper``). Holding the gains host-side lets
the app re-assert them the same way, so a tuned loop survives a reset **without
writing EEPROM at all**.

Kept deliberately separate from ``HardwareConfig`` because — like the camera
calibrations and the microscope config — this is a property of *this physical
rig*, not of a swappable print setup: a hardware-setup file loaded from
another machine must never carry another rig's heater wiring or ceiling
(the ``CAMERA_CAL_PERSIST_STORE`` lesson).

File: ``config/hardware/incubator.json``. ``MEBP_INCUBATOR_CONFIG_DIR``
redirects it for test isolation. Atomic writes (tmp + ``os.replace``);
unknown keys in an on-disk file are ignored; a partial/older file is merged
over the blank template so consumers always see a complete structure.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_FILENAME = "incubator.json"

#: Transports understood by the Incubator page / service.
#:   shared   — ride the app's live ZP connection (the heaters are wired to
#:              the SAME SKR Mini E3 V3 that drives Z + pumps on this rig)
#:   serial   — the incubator has its OWN Marlin board on its own COM port
#:   simulate — the built-in thermal simulator (no hardware)
TRANSPORTS = ("shared", "serial", "simulate")

#: The code-level ceiling (see ``incubator.safety.MAX_SETPOINT_C``). The store
#: may only LOWER it — a config file must never be able to raise the hard
#: limit above what the source ships with.
HARD_MAX_SETPOINT_C = 50.0

#: Ramp steps above this can arm Marlin's heat-up watchdog (see
#: ``incubator.ramp.MAX_SAFE_STEP_C``); the store clamps rather than trusts.
MAX_RAMP_STEP_C = 4.0

#: Plausibility bounds for stored PID gains. A gain outside these is refused
#: outright rather than clamped: a clamped gain is a *different controller*
#: than the one that was tuned, silently, on a live heater — absent is
#: recoverable (the board keeps its own gains), wrong is not.
PID_MAX_KP = 1000.0
PID_MAX_KI = 100.0
PID_MAX_KD = 10000.0

#: The two zone ids (``incubator.zones.ALL_ZONES``). Kept as data here so the
#: store stays importable without the rest of the package.
ZONE_IDS = ("bed", "hotend")


def _default_path() -> Path:
    d = os.environ.get("MEBP_INCUBATOR_CONFIG_DIR")
    if d:
        return Path(d) / _DEFAULT_FILENAME
    return Path("config/hardware") / _DEFAULT_FILENAME


def clamp_ceiling(value) -> float:
    """A usable setpoint ceiling in °C — never above the code hard max."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return HARD_MAX_SETPOINT_C
    return max(1.0, min(HARD_MAX_SETPOINT_C, v))


def clamp_ramp_step(value) -> float:
    """A watchdog-safe ramp step in °C."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 3.0
    return max(0.5, min(MAX_RAMP_STEP_C, v))


def parse_pid(value) -> Optional[dict]:
    """
    A usable ``{"kp","ki","kd"}`` from arbitrary stored/typed input, or None.

    Returns None — never a partially-repaired dict — when anything is missing,
    non-finite or out of :data:`PID_MAX_KP` / ``KI`` / ``KD`` range, so a
    malformed config leaves the board's own gains alone instead of pushing a
    controller nobody chose.
    """
    if not isinstance(value, dict):
        return None
    out = {}
    for key, hi in (("kp", PID_MAX_KP), ("ki", PID_MAX_KI), ("kd", PID_MAX_KD)):
        if key not in value:
            return None
        try:
            v = float(value[key])
        except (TypeError, ValueError):
            return None
        if v != v or v in (float("inf"), float("-inf")):
            return None
        if v < 0.0 or v > hi:
            return None
        out[key] = v
    if out["kp"] <= 0.0:
        return None      # a zero proportional gain is not a controller
    return out


def _blank_zone() -> dict:
    return {"label": "", "enabled": True, "preset_c": 37.0, "pid": None}


class IncubatorConfigStore:
    """Load/save the per-machine incubator configuration."""

    def __init__(self, path: Optional[Path] = None):
        self._path = Path(path) if path is not None else _default_path()
        self._lock = threading.RLock()
        self._data: dict = self._blank()
        self._load()

    # ── Defaults / load / save ────────────────────────────────────

    @staticmethod
    def _blank() -> dict:
        return {
            "transport": "shared",
            "dedicated_port": "",
            "dedicated_baud": 38400,
            "sim_time_scale": 300,
            "max_setpoint_c": HARD_MAX_SETPOINT_C,
            "ramp_step_c": 3.0,
            "fine_period_s": 60,
            "log_on_connect": False,
            "heaters_off_on_app_exit": True,
            #: Push the stored per-zone gains onto the board on every connect,
            #: and again whenever a board reset is detected. Costs one M304
            #: per zone and needs no EEPROM write.
            "pid_apply_on_connect": True,
            "zones": {zid: _blank_zone() for zid in ZONE_IDS},
        }

    def _load(self) -> None:
        with self._lock:
            if not self._path.exists():
                return
            try:
                with open(self._path, "r", encoding="utf-8") as f:
                    raw = json.load(f)
            except Exception as e:
                logger.warning("incubator config load failed (%s): %s",
                               self._path, e)
                return
            if not isinstance(raw, dict):
                return
            data = self._blank()
            for key in data:
                if key not in raw:
                    continue
                if key == "zones":
                    zones = raw.get("zones")
                    if isinstance(zones, dict):
                        for zid in ZONE_IDS:
                            z = zones.get(zid)
                            if isinstance(z, dict):
                                entry = data["zones"][zid]
                                if "label" in z:
                                    entry["label"] = str(z["label"] or "")
                                if "enabled" in z:
                                    entry["enabled"] = bool(z["enabled"])
                                if "preset_c" in z:
                                    try:
                                        entry["preset_c"] = clamp_ceiling(
                                            z["preset_c"])
                                    except Exception:
                                        pass
                                if "pid" in z:
                                    entry["pid"] = parse_pid(z["pid"])
                else:
                    data[key] = raw[key]
            # Validation belongs to the store, not the UI (one owner).
            if data.get("transport") not in TRANSPORTS:
                data["transport"] = "shared"
            data["max_setpoint_c"] = clamp_ceiling(data.get("max_setpoint_c"))
            data["ramp_step_c"] = clamp_ramp_step(data.get("ramp_step_c"))
            try:
                data["fine_period_s"] = max(
                    10, min(600, int(data.get("fine_period_s", 60))))
            except (TypeError, ValueError):
                data["fine_period_s"] = 60
            try:
                data["sim_time_scale"] = max(
                    1, min(5000, int(data.get("sim_time_scale", 300))))
            except (TypeError, ValueError):
                data["sim_time_scale"] = 300
            try:
                data["dedicated_baud"] = max(
                    1200, int(data.get("dedicated_baud", 38400)))
            except (TypeError, ValueError):
                data["dedicated_baud"] = 38400
            data["dedicated_port"] = str(data.get("dedicated_port") or "").strip()
            data["pid_apply_on_connect"] = bool(
                data.get("pid_apply_on_connect", True))
            self._data = data

    def _write(self) -> None:
        """Atomic write. Caller holds the lock."""
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_name(self._path.name + ".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self._data, f, indent=2)
        os.replace(tmp, self._path)

    def save(self) -> bool:
        with self._lock:
            try:
                self._write()
                return True
            except Exception as e:
                logger.warning("incubator config save failed (%s): %s",
                               self._path, e)
                return False

    # ── Generic access ────────────────────────────────────────────

    def get(self, key: str, default=None):
        with self._lock:
            return self._data.get(key, default)

    def set(self, key: str, value, *, save: bool = True) -> None:
        with self._lock:
            if key == "transport" and value not in TRANSPORTS:
                logger.warning("ignoring unknown incubator transport %r", value)
                return
            if key == "max_setpoint_c":
                value = clamp_ceiling(value)
            elif key == "ramp_step_c":
                value = clamp_ramp_step(value)
            self._data[key] = value
            if save:
                try:
                    self._write()
                except Exception as e:
                    logger.warning("incubator config save failed: %s", e)

    def as_dict(self) -> dict:
        with self._lock:
            return json.loads(json.dumps(self._data))

    # ── Zone helpers ──────────────────────────────────────────────

    def zone(self, zone_id: str) -> dict:
        """A copy of one zone's entry (blank defaults for an unknown id)."""
        with self._lock:
            z = (self._data.get("zones") or {}).get(zone_id)
            if isinstance(z, dict):
                return dict(z)
            return _blank_zone()

    def set_zone(self, zone_id: str, *, label=None, enabled=None,
                 preset_c=None, pid=..., save: bool = True) -> None:
        with self._lock:
            zones = self._data.setdefault("zones", {})
            entry = zones.setdefault(zone_id, _blank_zone())
            if label is not None:
                entry["label"] = str(label)
            if enabled is not None:
                entry["enabled"] = bool(enabled)
            if preset_c is not None:
                entry["preset_c"] = clamp_ceiling(preset_c)
            # `pid` uses an ellipsis sentinel, not None: None is the meaningful
            # value "forget the stored gains and leave the board alone".
            if pid is not ...:
                entry["pid"] = parse_pid(pid) if pid is not None else None
            if save:
                try:
                    self._write()
                except Exception as e:
                    logger.warning("incubator config save failed: %s", e)

    def zone_pid(self, zone_id: str) -> Optional[dict]:
        """Validated ``{"kp","ki","kd"}`` for a zone, or None if none stored."""
        return parse_pid(self.zone(zone_id).get("pid"))

    def zone_labels(self) -> dict[str, str]:
        """Non-empty operator labels keyed by zone id."""
        with self._lock:
            out = {}
            for zid, z in (self._data.get("zones") or {}).items():
                label = str((z or {}).get("label") or "").strip()
                if label:
                    out[zid] = label
            return out


# ── Module singleton ─────────────────────────────────────────────

_store: Optional[IncubatorConfigStore] = None
_store_lock = threading.Lock()


def get_store(path: Optional[Path] = None) -> IncubatorConfigStore:
    global _store
    with _store_lock:
        if _store is None:
            _store = IncubatorConfigStore(path)
        return _store


def reset_store() -> None:
    """Drop the singleton (test isolation after changing the env override)."""
    global _store
    with _store_lock:
        _store = None
