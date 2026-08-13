"""
telemetry.py — JSONL run logger for incubator holds.

Mirrors the house pattern from ``SupportClasses/PrintExecutionLogger.py``:
  * directory resolved from ``__file__``, never the CWD,
  * one JSON object per line, ``flush()`` after every write so a crash or power
    loss still leaves everything up to that instant on disk,
  * idempotent ``stop()``,
  * every public method swallows its exceptions — logging must never be able to
    take down a run that is heating real hardware.

JSONL (not CSV) matches the repo convention for telemetry (``logs/prints/``,
``logs/timing/``) and copes with the fact that the channel set is dynamic: the
phase-2 sensor box will add columns a CSV header could not have known about.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)


def _repo_root() -> Path:
    """
    Walk up looking for the repo root (the directory containing SupportClasses).
    Falls back to two levels up from this file, which is the layout we ship.
    """
    here = Path(__file__).resolve()
    for cand in (here.parent, *here.parents):
        if (cand / "SupportClasses").is_dir():
            return cand
    return here.parent.parent.parent


DEFAULT_LOG_DIR = _repo_root() / "logs" / "incubator"

#: Set MEBP_INCUBATOR_LOG=0 to disable telemetry entirely.
_ENV_KILL = "MEBP_INCUBATOR_LOG"


class TelemetryLogger:
    """One JSONL file per logging session."""

    def __init__(self, log_dir: Path | None = None):
        self.log_dir = Path(log_dir) if log_dir else DEFAULT_LOG_DIR
        self._file = None
        self._lock = threading.Lock()
        self._t0 = 0.0
        self._active = False
        self._stopped = False
        self.path: Path | None = None
        self._lines = 0

    # ── lifecycle ───────────────────────────────────────────────────

    @property
    def active(self) -> bool:
        return self._active

    @property
    def line_count(self) -> int:
        return self._lines

    def start(self, label: str = "hold", manifest: dict | None = None) -> Path | None:
        """Open a new log file. Returns its path, or ``None`` if disabled."""
        if os.environ.get(_ENV_KILL, "1") == "0":
            return None
        if self._active:
            return self.path
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe = "".join(
                ch if (ch.isalnum() or ch in "-_ ") else "_" for ch in str(label)
            ).strip()[:60] or "hold"
            self.path = self.log_dir / f"incu_{stamp}_{safe}.jsonl"
            self._file = open(self.path, "w", encoding="utf-8")
            self._t0 = time.monotonic()
            self._active = True
            self._stopped = False
            self._lines = 0
            self._write({"ev": "session_start",
                         "wall": datetime.now().isoformat(timespec="seconds"),
                         "label": safe,
                         "manifest": manifest or {}})
            return self.path
        except Exception as e:
            logger.warning("telemetry start failed: %s", e)
            self._file = None
            self._active = False
            return None

    def stop(self, reason: str = "stopped") -> None:
        """Idempotent close."""
        if self._stopped or not self._active:
            return
        self._stopped = True
        try:
            self._write({"ev": "session_end", "reason": reason,
                         "lines": self._lines})
        except Exception:
            pass
        with self._lock:
            try:
                if self._file is not None:
                    self._file.close()
            except Exception:
                pass
            self._file = None
            self._active = False

    # ── writing ─────────────────────────────────────────────────────

    def _write(self, obj: dict) -> None:
        with self._lock:
            f = self._file
            if f is None:
                return
            try:
                obj = {"t": round(time.monotonic() - self._t0, 3), **obj}
                f.write(json.dumps(obj, ensure_ascii=False, default=repr) + "\n")
                f.flush()
                self._lines += 1
            except Exception as e:
                logger.warning("telemetry write failed, disabling: %s", e)
                try:
                    f.close()
                except Exception:
                    pass
                self._file = None
                self._active = False

    def log_sample(self, channels: list, zones: dict | None = None) -> None:
        """
        One periodic sample. ``channels`` is a list of
        :class:`~.sensors.SensorChannel`; ``zones`` maps zone id -> a
        :class:`~.stability.StabilityReport`-shaped dict.
        """
        if not self._active:
            return
        try:
            self._write({
                "ev": "sample",
                "ch": {
                    c.uid: {
                        "raw": round(c.raw_c, 3),
                        "val": round(c.value_c, 3),
                        "tgt": None if c.target_c is None else round(c.target_c, 2),
                        "duty": None if c.power_pct is None else round(c.power_pct, 1),
                        "cal": c.calibrated,
                        "stale": c.stale,
                    }
                    for c in channels
                },
                "zones": zones or {},
            })
        except Exception:
            pass

    def log_event(self, event: str, /, **fields) -> None:
        """
        Any discrete event: setpoint change, autotune, fault, command.

        ``event`` is POSITIONAL-ONLY on purpose. Callers naturally want to pass
        payload keys like ``kind=``, ``type=`` or ``event=``, and any of those
        would collide with the parameter name and raise
        "got multiple values for argument" at runtime — inside a background
        thread, where the exception is easy to lose. The ``/`` makes that
        impossible for every present and future call site.
        """
        if not self._active:
            return
        try:
            self._write({"ev": event, **fields})
        except Exception:
            pass
