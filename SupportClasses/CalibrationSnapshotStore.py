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

from SupportClasses.MachineConfig import resolve_machine_path

logger = logging.getLogger(__name__)

_DEFAULT_PATH = resolve_machine_path("last_calibration.json")

# Fingerprint dimensions → human label, in the order they're reported.
_FINGERPRINT_LABELS = {
    "device": "device profile",
    "plate_format": "plate format",
    "needle_gauge": "needle gauge",
    "needle_type": "needle type",
    "needle_bore_um": "needle bore",
    "needle_tip_length_mm": "needle tip length",
    "needle_form": "needle form",
    "needle_bore_count": "needle bore count",
    "axis_map": "axis map",
    "steps_per_mm": "steps/mm",
    "plate_flip_180": "plate orientation",
}


def _bore_count(get) -> Optional[int]:
    """How many bores the saved needle has, from a ``settings.get``.

    ``bores`` is conditional-emit (present only for a genuine multi-bore
    assembly), so this reads the list when it is there and otherwise falls back
    to ``num_channels``, which is always emitted. Returns None only when there is
    no needle section at all — an absent value must stay absent so
    :meth:`fingerprint_diff`'s added-dimension rule applies.
    """
    bores = get("hardware_config.needle.bores")
    if isinstance(bores, list) and bores:
        return len(bores)
    n = get("hardware_config.needle.num_channels")
    try:
        return max(1, int(n))
    except (TypeError, ValueError):
        return None


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
            # v7.6: a pulled glass capillary has NO gauge, so gauge alone would
            # fingerprint every capillary identically and silently trust a stale
            # calibration across a tip change. The bore (tip Ø when pulled) and
            # the tip length identify it — and the tip length matters twice
            # over, because a pulled needle is barrel + tip long, so changing it
            # invalidates the plate-bottom Z touch-off exactly as changing the
            # barrel length does.
            "needle_type": g("hardware_config.needle.needle_type"),
            "needle_bore_um": (g("hardware_config.needle.tip_id_um")
                               or g("hardware_config.needle.id_um")),
            "needle_tip_length_mm": g("hardware_config.needle.tip_length_mm"),
            # v7.9: the ASSEMBLY, not just bore 0. Every dimension above
            # describes ONE bore (they mirror the flat fields = bore 0), so
            # swapping a single needle for a backpack — or a backpack for a
            # triple — changes nothing they can see, while it changes the tip
            # that touched off the plate bottom AND invalidates every measured
            # bore mount offset (NeedleBoreCalibrationStore). Bore count falls
            # back to num_channels, which every saved setup carries, so a
            # pre-v7.9 config reports 1 rather than None.
            "needle_form": g("hardware_config.needle.needle_form"),
            "needle_bore_count": _bore_count(g),
            # v7.5.x: plate orientation — a change mirrors the well→stage
            # mapping, so a saved calibration must not be trusted across it.
            "plate_flip_180": g("device_profile.plate_flip_180"),
        }

    @staticmethod
    def fingerprint_diff(saved: Optional[dict], current: Optional[dict]) -> list:
        """Human-readable list of changed dimensions (empty ⇒ unchanged).

        A missing saved fingerprint (older snapshot) reports no diff so we
        don't manufacture a scary warning from absent data. Likewise a
        dimension ADDED after the snapshot was written is skipped: it is absent
        on the saved side and populated on the current side, which would
        otherwise fire a bogus "needle bore: None → 210" for every user the
        first time they launch a build that adds a dimension.
        """
        if not saved:
            return []
        current = current or {}
        diffs = []
        for key, label in _FINGERPRINT_LABELS.items():
            if key not in saved:
                continue          # dimension added after this snapshot
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
