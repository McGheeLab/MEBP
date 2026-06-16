"""
CalibrationSnapshotStore.py — Last-known-good calibration checkpoint.

v7.5.x: After a software restart with an *unchanged* physical setup, operators
had to redo the entire calibration (needle zero + plate wells/warp + Z plane &
reference heights). The three layers were persisted independently and could be
silently lost — the live ``settings.json → calibration`` section is rewritten
from in-memory ``CalibrationPage`` state on a debounced auto-save, so a spurious
plate-format change that cleared the page wiped the saved section to null; the
needle ``zero_position`` was only persisted on an explicit Set Zero or a clean
shutdown.

This store keeps a single, **stable** "last known good" snapshot — bundling
needle zero + plate + Z + a hardware fingerprint + a timestamp — in its own
file, so it is immune to that live-state clobber and can be restored as a unit.
It is written only when a real plate calibration is present and is never
overwritten by an empty state. On the next launch the GUI offers to restore it
(``MainWindow._maybe_prompt_calibration_restore``, mirroring the ZP
position-restore prompt) when the live calibration came up empty.

Scope: needle zero + plate + Z only. Camera/objective µm/px keep their own
identity-keyed stores (``CameraCalibrationStore`` / ``ObjectiveCalibration``).

Data file: config/hardware/last_calibration.json

Structure::

    {
      "version": "1.0",
      "saved_at": "2026-06-15T17:30:00",
      "fingerprint": {
        "device": "ME3B V1",
        "axis_map": {"Z": "Z", "P1": "X", "P2": "Y", "P3": "E"},
        "steps_per_mm": {"Z": 5255.0, "P1": 10120.0, ...},
        "plate_format": 24,
        "needle_gauge": 27
      },
      "zero_position": {"x": ..., "y": ..., "Z": ..., "P1": ..., "P2": ..., "P3": ...},
      "calibration": { ... CalibrationPage._save_calibration payload ... }
    }
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_PATH = Path("config/hardware/last_calibration.json")

# Fingerprint dimensions → human label, in the order they're reported.
_FINGERPRINT_LABELS = {
    "device": "device profile",
    "plate_format": "plate format",
    "needle_gauge": "needle gauge",
    "axis_map": "axis map",
    "steps_per_mm": "steps/mm",
}


class CalibrationSnapshotStore:
    """Load/save the single last-known-good calibration snapshot."""

    def __init__(self, path: Path = _DEFAULT_PATH):
        self._path = Path(path)
        self._data: Optional[dict] = None
        self._load()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if not self._path.exists():
            self._data = None
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            self._data = loaded if isinstance(loaded, dict) else None
        except Exception as exc:
            logger.warning(
                f"CalibrationSnapshotStore: failed to load {self._path}: {exc}")
            self._data = None

    def save_snapshot(
        self,
        zero_position: dict,
        calibration: dict,
        fingerprint: Optional[dict] = None,
        saved_at: Optional[str] = None,
    ) -> dict:
        """Write the snapshot atomically (temp + os.replace) and return it.

        Callers should only invoke this when a real calibration exists (the
        snapshot is a known-good checkpoint); this method does not itself gate
        on content beyond coercing the inputs to JSON-safe primitives.
        """
        snap = {
            "version": "1.0",
            "saved_at": saved_at or datetime.now().isoformat(timespec="seconds"),
            "fingerprint": dict(fingerprint or {}),
            "zero_position": {
                k: (round(float(v), 4) if isinstance(v, (int, float)) else v)
                for k, v in (zero_position or {}).items()
            },
            "calibration": dict(calibration or {}),
        }
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_name(self._path.name + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(snap, f, indent=2)
            os.replace(tmp, self._path)  # atomic on the same filesystem
            self._data = snap
            logger.info(
                f"Calibration snapshot saved ({snap['saved_at']}) → {self._path}")
        except Exception as exc:
            logger.error(f"CalibrationSnapshotStore: failed to save: {exc}")
        return snap

    def load_snapshot(self) -> Optional[dict]:
        """Return the stored snapshot dict, or None if there isn't one."""
        return self._data

    def has_snapshot(self) -> bool:
        return isinstance(self._data, dict) and bool(self._data.get("calibration"))

    def clear(self) -> None:
        self._data = None
        try:
            if self._path.exists():
                self._path.unlink()
        except Exception as exc:
            logger.warning(f"CalibrationSnapshotStore: failed to clear: {exc}")

    # ── Hardware fingerprint ──────────────────────────────────────

    @staticmethod
    def build_fingerprint(settings) -> dict:
        """Capture the setup dimensions that, if changed, make a saved
        calibration suspect. ``settings`` is anything with a
        ``get(dotpath, default=None)`` (the app's ``Settings``)."""
        g = settings.get
        return {
            "device": g("device_profile.active"),
            "axis_map": dict(g("device_profile.axis_map") or {}),
            "steps_per_mm": dict(g("device_profile.steps_per_mm") or {}),
            "plate_format": g("hardware_config.plate_format",
                              g("calibration.plate_format")),
            "needle_gauge": g("hardware_config.needle.gauge"),
        }

    @staticmethod
    def fingerprint_diff(saved: Optional[dict], current: Optional[dict]) -> list:
        """Human-readable list of changed dimensions (empty ⇒ unchanged).

        A missing saved fingerprint (older snapshot) reports no diff so we
        don't manufacture a scary warning from absent data.
        """
        if not saved:
            return []
        current = current or {}
        diffs = []
        for key, label in _FINGERPRINT_LABELS.items():
            s = saved.get(key)
            c = current.get(key)
            if s != c:
                diffs.append(f"{label}: {s} → {c}")
        return diffs


# ── Module-level singleton ──────────────────────────────────────────

_store: Optional[CalibrationSnapshotStore] = None


def get_store(path: Path = _DEFAULT_PATH) -> CalibrationSnapshotStore:
    """Return the module-level singleton CalibrationSnapshotStore."""
    global _store
    if _store is None:
        _store = CalibrationSnapshotStore(path)
    return _store
