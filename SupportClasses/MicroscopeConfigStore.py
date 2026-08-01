"""
MicroscopeConfigStore.py — per-machine motorized-microscope settings (v7.5.x).

Owns everything the operator configures about the microscope BODY (the Nikon Ti
Eclipse turrets + focus drive), kept deliberately separate from
``HardwareConfig`` because — like the camera calibrations and the calibration
status — it is a property of *this physical rig*, not of a swappable print
setup. A hardware-setup file loaded from another machine must never carry
another rig's filter-cube assignments.

What lives here:

* **backend selection** — which driver talks to the body (``simulated`` /
  ``nikon_ti`` / ``micromanager``) plus the few driver-specific knobs
  (COM ProgID, Micro-Manager config path, Z device-units-per-µm).
* **slot assignments** — the operator's name for each filter-cube slot and each
  nosepiece position. The turret reports a POSITION NUMBER; only the operator
  knows that slot 3 holds the mCherry cube. That mapping is exactly what this
  store exists to remember.
* **focus preferences** — jog step, direction convention, optional soft limits.

Data file: ``config/hardware/microscope.json`` (override the directory with
``$MEBP_MICROSCOPE_CONFIG_DIR`` for test isolation). Atomic writes
(temp + ``os.replace``), same as ``MosaicStore`` / ``CalibrationStatusStore``.

Slot numbering is **1-based everywhere** — it matches the numbers engraved on
the turret, so what the operator reads on the microscope is what the UI shows.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_FILENAME = "microscope.json"

#: Backend identifiers understood by :mod:`SupportClasses.MicroscopeControl`.
BACKENDS = ("simulated", "nikon_ti", "micromanager")

#: A Ti filter cassette holds 6 cubes; the nosepiece is commonly 6 (some are 5).
_DEFAULT_FILTER_SLOTS = 6
_DEFAULT_OBJECTIVE_SLOTS = 6

#: Nikon Ti ZDrive counts in 10 nm units → 100 units per µm. Bench-verifiable
#: (command a 100 µm move, measure it) and overridable per machine, because
#: getting this wrong scales every focus move.
_DEFAULT_Z_UNITS_PER_UM = 100.0

_DEFAULT_FOCUS_STEP_UM = 10.0

_MAX_SLOTS = 12  # sanity ceiling; no microscope turret is anywhere near this


def _default_path() -> Path:
    d = os.environ.get("MEBP_MICROSCOPE_CONFIG_DIR")
    if d:
        return Path(d) / _DEFAULT_FILENAME
    return Path("config/hardware") / _DEFAULT_FILENAME


def _clean_slots(value, default: int) -> int:
    try:
        n = int(value)
    except (TypeError, ValueError):
        return default
    return max(1, min(_MAX_SLOTS, n))


class MicroscopeConfigStore:
    """Load/save the per-machine microscope configuration."""

    def __init__(self, path: Optional[Path] = None):
        self._path = Path(path) if path is not None else _default_path()
        self._lock = threading.RLock()
        self._data: dict = self._blank()
        self._load()

    # ── Defaults / load / save ────────────────────────────────────

    @staticmethod
    def _blank() -> dict:
        return {
            "version": "1.0",
            "backend": "simulated",
            # Driver-specific knobs (only the selected backend reads its own).
            "prog_id": "",              # Ti SDK COM ProgID override
            "cassette": 1,              # which filter cassette on a dual-cube body
            "z_units_per_um": _DEFAULT_Z_UNITS_PER_UM,
            "mm_config_path": "",       # Micro-Manager .cfg
            "mm_dir": "",               # Micro-Manager install (device adapters)
            # Device names as published by Micro-Manager's NikonTI adapter
            # (https://micro-manager.org/NikonTI). They hang off the TIScope hub,
            # which the .cfg must load first. Editable — a config may relabel them.
            "mm_filter_device": "TIFilterBlock1",
            "mm_objective_device": "TINosePiece",
            "mm_focus_device": "TIZDrive",
            # Slot assignments (operator-owned names).
            "filter_slots": _DEFAULT_FILTER_SLOTS,
            "objective_slots": _DEFAULT_OBJECTIVE_SLOTS,
            "filter_cubes": {},         # {"1": "DAPI", ...} — 1-based string keys
            "objectives": {},           # {"1": "4x Plan Fluor", ...}
            # Focus preferences.
            "focus_step_um": _DEFAULT_FOCUS_STEP_UM,
            "focus_up_is_positive": True,
            "focus_min_um": None,       # optional operator soft limits
            "focus_max_um": None,
        }

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
        except Exception as exc:
            logger.warning(
                f"MicroscopeConfigStore: failed to load {self._path}: {exc}")
            return
        if not isinstance(loaded, dict):
            return
        # Merge over the blank template so a partial / older file still yields a
        # complete, well-typed structure.
        data = self._blank()
        for key, value in loaded.items():
            if key not in data:
                continue  # ignore unknown keys rather than choking on them
            data[key] = value
        data["backend"] = (data.get("backend")
                           if data.get("backend") in BACKENDS else "simulated")
        data["filter_slots"] = _clean_slots(
            data.get("filter_slots"), _DEFAULT_FILTER_SLOTS)
        data["objective_slots"] = _clean_slots(
            data.get("objective_slots"), _DEFAULT_OBJECTIVE_SLOTS)
        for key in ("filter_cubes", "objectives"):
            raw = data.get(key)
            data[key] = ({str(k): str(v) for k, v in raw.items()}
                         if isinstance(raw, dict) else {})
        self._data = data

    def _write(self) -> None:
        """Atomic write. Caller holds ``_lock``."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_name(self._path.name + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
        except Exception as exc:
            logger.error(f"MicroscopeConfigStore: failed to save: {exc}")

    def save(self) -> None:
        with self._lock:
            self._write()

    # ── Generic access ────────────────────────────────────────────

    def get(self, key: str, default=None):
        with self._lock:
            return self._data.get(key, default)

    def set(self, key: str, value, *, save: bool = True) -> None:
        with self._lock:
            self._data[key] = value
            if save:
                self._write()

    def as_dict(self) -> dict:
        with self._lock:
            return json.loads(json.dumps(self._data))  # deep copy

    # ── Backend ───────────────────────────────────────────────────

    def get_backend(self) -> str:
        with self._lock:
            b = self._data.get("backend", "simulated")
            return b if b in BACKENDS else "simulated"

    def set_backend(self, backend: str) -> None:
        if backend not in BACKENDS:
            raise ValueError(f"unknown microscope backend: {backend!r}")
        self.set("backend", backend)

    def backend_kwargs(self) -> dict:
        """Constructor kwargs for the selected backend."""
        with self._lock:
            d = self._data
            backend = self.get_backend()
            if backend == "nikon_ti":
                return {
                    "prog_id": (d.get("prog_id") or None),
                    "z_units_per_um": float(
                        d.get("z_units_per_um") or _DEFAULT_Z_UNITS_PER_UM),
                    "cassette": int(d.get("cassette") or 1),
                }
            if backend == "micromanager":
                return {
                    "config_path": d.get("mm_config_path") or "",
                    "mm_dir": (d.get("mm_dir") or None),
                    "filter_device": d.get("mm_filter_device")
                    or "TIFilterBlock1",
                    "objective_device": d.get("mm_objective_device")
                    or "TINosePiece",
                    "focus_device": d.get("mm_focus_device") or "TIZDrive",
                }
            return {
                "filter_slots": int(d.get("filter_slots")
                                    or _DEFAULT_FILTER_SLOTS),
                "objective_slots": int(d.get("objective_slots")
                                       or _DEFAULT_OBJECTIVE_SLOTS),
            }

    # ── Slot counts ───────────────────────────────────────────────

    def filter_slots(self) -> int:
        with self._lock:
            return _clean_slots(self._data.get("filter_slots"),
                                _DEFAULT_FILTER_SLOTS)

    def objective_slots(self) -> int:
        with self._lock:
            return _clean_slots(self._data.get("objective_slots"),
                                _DEFAULT_OBJECTIVE_SLOTS)

    def set_filter_slots(self, n: int) -> None:
        self.set("filter_slots", _clean_slots(n, _DEFAULT_FILTER_SLOTS))

    def set_objective_slots(self, n: int) -> None:
        self.set("objective_slots", _clean_slots(n, _DEFAULT_OBJECTIVE_SLOTS))

    # ── Slot assignments ──────────────────────────────────────────

    def _labels(self, key: str, count: int) -> dict[int, str]:
        with self._lock:
            raw = self._data.get(key) or {}
            out: dict[int, str] = {}
            for pos in range(1, count + 1):
                name = str(raw.get(str(pos), "") or "").strip()
                if name:
                    out[pos] = name
            return out

    def filter_labels(self) -> dict[int, str]:
        """``{position: name}`` for every *named* filter slot (1-based)."""
        return self._labels("filter_cubes", self.filter_slots())

    def objective_labels(self) -> dict[int, str]:
        """``{position: name}`` for every *named* nosepiece position (1-based)."""
        return self._labels("objectives", self.objective_slots())

    def filter_label(self, position: int) -> str:
        return self.filter_labels().get(int(position), "")

    def objective_label(self, position: int) -> str:
        return self.objective_labels().get(int(position), "")

    def set_filter_label(self, position: int, name: str, *,
                         save: bool = True) -> None:
        self._set_label("filter_cubes", position, name, save=save)

    def set_objective_label(self, position: int, name: str, *,
                            save: bool = True) -> None:
        self._set_label("objectives", position, name, save=save)

    def _set_label(self, key: str, position: int, name: str, *,
                   save: bool) -> None:
        pos = int(position)
        if pos < 1:
            raise ValueError(f"slot positions are 1-based, got {position!r}")
        clean = str(name or "").strip()
        with self._lock:
            table = self._data.setdefault(key, {})
            if clean:
                table[str(pos)] = clean
            else:
                table.pop(str(pos), None)  # blank clears the assignment
            if save:
                self._write()

    def set_filter_labels(self, labels: dict) -> None:
        """Replace every filter-cube assignment in one write."""
        self._set_labels("filter_cubes", labels)

    def set_objective_labels(self, labels: dict) -> None:
        """Replace every nosepiece assignment in one write."""
        self._set_labels("objectives", labels)

    def _set_labels(self, key: str, labels: dict) -> None:
        table = {}
        for pos, name in (labels or {}).items():
            try:
                p = int(pos)
            except (TypeError, ValueError):
                continue
            clean = str(name or "").strip()
            if p >= 1 and clean:
                table[str(p)] = clean
        with self._lock:
            self._data[key] = table
            self._write()

    # ── Focus preferences ─────────────────────────────────────────

    def focus_step_um(self) -> float:
        with self._lock:
            try:
                return max(0.001, float(self._data.get("focus_step_um")
                                        or _DEFAULT_FOCUS_STEP_UM))
            except (TypeError, ValueError):
                return _DEFAULT_FOCUS_STEP_UM

    def set_focus_step_um(self, step: float) -> None:
        self.set("focus_step_um", max(0.001, float(step)))

    def focus_up_is_positive(self) -> bool:
        with self._lock:
            return bool(self._data.get("focus_up_is_positive", True))

    def set_focus_up_is_positive(self, value: bool) -> None:
        self.set("focus_up_is_positive", bool(value))

    def focus_soft_limits_um(self) -> tuple[Optional[float], Optional[float]]:
        """Operator soft limits ``(min, max)``; either may be ``None``."""
        with self._lock:
            def _f(key):
                v = self._data.get(key)
                if v is None or v == "":
                    return None
                try:
                    return float(v)
                except (TypeError, ValueError):
                    return None
            lo, hi = _f("focus_min_um"), _f("focus_max_um")
        if lo is not None and hi is not None and lo > hi:
            lo, hi = hi, lo
        return lo, hi

    def set_focus_soft_limits_um(self, lo: Optional[float],
                                 hi: Optional[float]) -> None:
        with self._lock:
            self._data["focus_min_um"] = None if lo is None else float(lo)
            self._data["focus_max_um"] = None if hi is None else float(hi)
            self._write()


# ── Module-level singleton ──────────────────────────────────────────

_store: Optional[MicroscopeConfigStore] = None


def get_store(path: Optional[Path] = None) -> MicroscopeConfigStore:
    """Return the module-level singleton :class:`MicroscopeConfigStore`."""
    global _store
    if _store is None:
        _store = MicroscopeConfigStore(path)
    return _store


def reset_store() -> None:
    """Drop the singleton (test isolation after changing the env override)."""
    global _store
    _store = None
