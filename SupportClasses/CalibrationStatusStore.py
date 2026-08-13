"""
CalibrationStatusStore.py — Calibration freshness / usability tracking.

v7.5.x: Operators asked for a "usability" pop-up that tells them when the XY, Z,
and pump (P) axes were last calibrated, and warns them to recalibrate when the
XY stage has traveled a long way since the last XY calibration, or when a long
time has elapsed since a calibration (``coding plans/changes.txt`` — Cason 7.21).

This store is the single per-machine owner of that state:

* a **lifetime XY travel odometer** (µm) accumulated from position-poller deltas,
* a **per-type "last calibrated at" timestamp** for ``xy`` / ``z`` / ``p`` (with
  the odometer snapshotted at the last XY calibration so "travel since XY cal"
  can be derived), and
* the two **thresholds** the pop-up compares against — ``xy_recal_travel_mm``
  (M) and ``recal_interval_hours`` (H). They live here (not on ``HardwareConfig``)
  because only the pop-up reads them, so there is nothing to fan out; the store
  owns everything about calibration status.

It is deliberately independent of the live ``settings.json`` auto-save so a
spurious page-state clobber can't wipe it (mirrors ``CalibrationSnapshotStore``).

Data file: ``config/hardware/calibration_status.json`` (override the directory
with ``$MEBP_CALIBRATION_STATUS_DIR`` for test isolation).

Structure::

    {
      "version": "1.0",
      "thresholds": {"xy_recal_travel_mm": 1000.0, "recal_interval_hours": 168.0},
      "xy_travel_um": 0.0,
      "xy": {"at": "2026-07-21T12:00:00", "travel_um": 0.0},
      "z":  {"at": "2026-07-21T12:00:00"},
      "p":  {"at": "2026-07-21T12:00:00"}
    }
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

# Calibration types tracked. "p" (pump) is a single most-recent-pump stamp — any
# pump's plunger setup updates it — matching the operator's XY/Z/P granularity.
KINDS = ("xy", "z", "p")

# Defaults for the two thresholds.
_DEFAULT_XY_RECAL_TRAVEL_MM = 1000.0   # M: ~1 m of XY travel since the last XY cal
_DEFAULT_RECAL_INTERVAL_HOURS = 168.0  # H: one week

# A single poller-sample XY chord larger than this (µm) is treated as a frame
# discontinuity (corrupt read / coordinate re-zero), not real travel, and
# dropped. 1 m dwarfs any real single move — even a full envelope traversal
# accumulated while the poller was suspended.
_ODOM_MAX_STEP_UM = 1_000_000.0

# Chords below this (µm) are dropped as encoder jitter at rest — otherwise a
# stationary stage could slowly inflate the odometer over a long session. Real
# XY moves are tens of µm and up, so this loses nothing meaningful.
_ODOM_MIN_STEP_UM = 1.0

# Throttle disk writes from the high-frequency odometer accumulation.
_SAVE_INTERVAL_S = 15.0

_DEFAULT_FILENAME = "calibration_status.json"


def _default_path() -> Path:
    d = os.environ.get("MEBP_CALIBRATION_STATUS_DIR")
    if d:
        return Path(d) / _DEFAULT_FILENAME
    return resolve_machine_path(_DEFAULT_FILENAME)


class CalibrationStatusStore:
    """Load/save the per-machine calibration-status state."""

    def __init__(self, path: Optional[Path] = None):
        self._path = Path(path) if path is not None else _default_path()
        self._lock = threading.RLock()
        self._data: dict = self._blank()
        self._dirty = False
        self._last_save_monotonic = 0.0
        self._load()

    # ── Defaults / load ───────────────────────────────────────────

    @staticmethod
    def _blank() -> dict:
        return {
            "version": "1.0",
            "thresholds": {
                "xy_recal_travel_mm": _DEFAULT_XY_RECAL_TRAVEL_MM,
                "recal_interval_hours": _DEFAULT_RECAL_INTERVAL_HOURS,
            },
            "xy_travel_um": 0.0,
            "xy": {"at": None, "travel_um": 0.0},
            "z": {"at": None},
            "p": {"at": None},
        }

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            with open(self._path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                # Merge over the blank template so a partial / older file still
                # yields a complete, well-typed structure.
                data = self._blank()
                th = loaded.get("thresholds")
                if isinstance(th, dict):
                    data["thresholds"].update(
                        {k: v for k, v in th.items()
                         if k in data["thresholds"]})
                try:
                    data["xy_travel_um"] = max(
                        0.0, float(loaded.get("xy_travel_um", 0.0)))
                except (TypeError, ValueError):
                    pass
                for kind in KINDS:
                    entry = loaded.get(kind)
                    if isinstance(entry, dict):
                        at = entry.get("at")
                        data[kind]["at"] = at if isinstance(at, str) else None
                        if kind == "xy":
                            try:
                                data["xy"]["travel_um"] = float(
                                    entry.get("travel_um", 0.0))
                            except (TypeError, ValueError):
                                data["xy"]["travel_um"] = 0.0
                self._data = data
        except Exception as exc:
            logger.warning(
                f"CalibrationStatusStore: failed to load {self._path}: {exc}")

    # ── Persistence ───────────────────────────────────────────────

    def _write(self) -> None:
        """Atomic write (temp + os.replace). Caller holds ``_lock``."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_name(self._path.name + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2)
            os.replace(tmp, self._path)
            self._dirty = False
            self._last_save_monotonic = time.monotonic()
        except Exception as exc:
            logger.error(f"CalibrationStatusStore: failed to save: {exc}")

    def flush(self) -> None:
        """Force-persist any pending changes (e.g. on shutdown)."""
        with self._lock:
            if self._dirty:
                self._write()

    def _maybe_save(self) -> None:
        """Persist if the throttle window has elapsed. Caller holds ``_lock``."""
        if not self._dirty:
            return
        if time.monotonic() - self._last_save_monotonic >= _SAVE_INTERVAL_S:
            self._write()

    # ── Odometer ──────────────────────────────────────────────────

    def add_xy_travel_um(self, delta_um: float) -> None:
        """Accumulate an XY travel segment (µm). Thread-safe + throttled.

        Segments beyond ``_ODOM_MAX_STEP_UM`` (frame discontinuities), below
        ``_ODOM_MIN_STEP_UM`` (jitter at rest), and non-finite / non-positive
        deltas are ignored.
        """
        try:
            d = float(delta_um)
        except (TypeError, ValueError):
            return
        if not (d >= _ODOM_MIN_STEP_UM) or d > _ODOM_MAX_STEP_UM:
            return
        with self._lock:
            self._data["xy_travel_um"] = float(
                self._data.get("xy_travel_um", 0.0)) + d
            self._dirty = True
            self._maybe_save()

    def get_xy_travel_um(self) -> float:
        with self._lock:
            return float(self._data.get("xy_travel_um", 0.0))

    def xy_travel_since_cal_um(self) -> Optional[float]:
        """µm traveled since the last XY calibration, or ``None`` if XY was
        never calibrated."""
        with self._lock:
            if not self._data["xy"].get("at"):
                return None
            snap = float(self._data["xy"].get("travel_um", 0.0))
            return max(0.0, float(self._data.get("xy_travel_um", 0.0)) - snap)

    # ── Timestamps ────────────────────────────────────────────────

    def mark_calibrated(self, kind: str, when: Optional[str] = None) -> None:
        """Stamp ``kind`` (∈ ``KINDS``) as calibrated now (or at ``when``).

        For ``xy`` the current odometer is snapshotted so travel-since-cal
        resets to zero. Persists immediately (calibration is an infrequent,
        deliberate action).
        """
        if kind not in KINDS:
            raise ValueError(f"unknown calibration kind: {kind!r}")
        stamp = when or datetime.now().isoformat(timespec="seconds")
        with self._lock:
            self._data[kind]["at"] = stamp
            if kind == "xy":
                self._data["xy"]["travel_um"] = float(
                    self._data.get("xy_travel_um", 0.0))
            self._dirty = True
            self._write()

    def get_calibrated_at(self, kind: str) -> Optional[str]:
        with self._lock:
            return self._data.get(kind, {}).get("at")

    def hours_since(self, kind: str, now: Optional[datetime] = None
                    ) -> Optional[float]:
        """Hours since ``kind`` was last calibrated, or ``None`` if never
        calibrated / the stored stamp can't be parsed."""
        at = self.get_calibrated_at(kind)
        if not at:
            return None
        try:
            then = datetime.fromisoformat(at)
        except (TypeError, ValueError):
            return None
        ref = now or datetime.now()
        return max(0.0, (ref - then).total_seconds() / 3600.0)

    # ── Thresholds (M / H) ────────────────────────────────────────

    def get_thresholds(self) -> tuple[float, float]:
        """Return ``(xy_recal_travel_mm, recal_interval_hours)``."""
        with self._lock:
            th = self._data["thresholds"]
            return (float(th.get("xy_recal_travel_mm",
                                 _DEFAULT_XY_RECAL_TRAVEL_MM)),
                    float(th.get("recal_interval_hours",
                                 _DEFAULT_RECAL_INTERVAL_HOURS)))

    def set_thresholds(self, m_mm: Optional[float] = None,
                       h_hours: Optional[float] = None) -> None:
        """Set either/both thresholds (mm, hours). Negatives clamp to 0
        (= disabled). Persists immediately."""
        with self._lock:
            th = self._data["thresholds"]
            if m_mm is not None:
                try:
                    th["xy_recal_travel_mm"] = max(0.0, float(m_mm))
                except (TypeError, ValueError):
                    pass
            if h_hours is not None:
                try:
                    th["recal_interval_hours"] = max(0.0, float(h_hours))
                except (TypeError, ValueError):
                    pass
            self._dirty = True
            self._write()


# ── Module-level singleton ──────────────────────────────────────────

_store: Optional[CalibrationStatusStore] = None


def get_store(path: Optional[Path] = None) -> CalibrationStatusStore:
    """Return the module-level singleton ``CalibrationStatusStore``."""
    global _store
    if _store is None:
        _store = CalibrationStatusStore(path)
    return _store
