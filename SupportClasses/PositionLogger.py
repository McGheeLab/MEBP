"""
Position Logger — Timestamped position recording for reproducibility.

Records stage positions, events, and metadata during calibration and
printing.  Supports export to CSV and JSON for post-processing.

Thread-safe via an explicit lock (not relying on CPython GIL).

Usage::

    plog = PositionLogger()
    plog.record("calibrate_zero", xy_pos=(1000, 2000, 0), zp_pos=(0.1, 0, 0, 0))
    plog.save_csv("log.csv")
"""

from __future__ import annotations

import csv
import json
import logging
import threading
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class PositionRecord:
    """A single timestamped position snapshot."""

    timestamp: str
    event: str
    xy_x: Optional[float] = None
    xy_y: Optional[float] = None
    xy_f: Optional[float] = None
    z: Optional[float] = None
    p1: Optional[float] = None
    p2: Optional[float] = None
    p3: Optional[float] = None
    metadata: dict = field(default_factory=dict)

    # CSV column order
    _CSV_FIELDS = (
        "timestamp", "event",
        "xy_x", "xy_y", "xy_f",
        "z", "p1", "p2", "p3",
        "metadata",
    )

    def to_flat_dict(self) -> dict:
        """Flatten for CSV export (metadata serialised as JSON string)."""
        return {
            "timestamp": self.timestamp,
            "event": self.event,
            "xy_x": self.xy_x,
            "xy_y": self.xy_y,
            "xy_f": self.xy_f,
            "z": self.z,
            "p1": self.p1,
            "p2": self.p2,
            "p3": self.p3,
            "metadata": json.dumps(self.metadata) if self.metadata else "",
        }


class PositionLogger:
    """
    Append-only log of position records with export capabilities.

    Thread-safe — multiple threads may call :meth:`record` concurrently.
    """

    def __init__(self):
        self._records: list[PositionRecord] = []
        self._lock = threading.Lock()

    # ── Recording ─────────────────────────────────────────────────

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._records)

    @property
    def records(self) -> list[PositionRecord]:
        """Return a shallow copy of all records."""
        with self._lock:
            return list(self._records)

    def record(
        self,
        event: str,
        xy_pos: Optional[tuple] = None,
        zp_pos: Optional[tuple] = None,
        metadata: Optional[dict] = None,
    ) -> None:
        """
        Append a position record.

        Args:
            event:    Event name (e.g. "calibrate_zero", "print_start").
            xy_pos:   (x, y, f) from XY stage, or None.
            zp_pos:   (z, p1, p2, p3) from ZP stage, or None.
            metadata: Optional dict of extra data.
        """
        now = datetime.now().isoformat(timespec="milliseconds")
        rec = PositionRecord(timestamp=now, event=event, metadata=metadata or {})

        if xy_pos is not None:
            rec.xy_x = xy_pos[0] if len(xy_pos) > 0 else None
            rec.xy_y = xy_pos[1] if len(xy_pos) > 1 else None
            rec.xy_f = xy_pos[2] if len(xy_pos) > 2 else None

        if zp_pos is not None:
            rec.z = zp_pos[0] if len(zp_pos) > 0 else None
            rec.p1 = zp_pos[1] if len(zp_pos) > 1 else None
            rec.p2 = zp_pos[2] if len(zp_pos) > 2 else None
            rec.p3 = zp_pos[3] if len(zp_pos) > 3 else None

        with self._lock:
            self._records.append(rec)

        logger.debug(f"Position logged: {event}")

    def clear(self) -> None:
        """Discard all records."""
        with self._lock:
            self._records.clear()
        logger.info("Position log cleared")

    # ── Export ─────────────────────────────────────────────────────

    def save_csv(self, filepath: str) -> None:
        """Write all records to a CSV file."""
        path = Path(filepath)
        with self._lock:
            snapshot = list(self._records)

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=PositionRecord._CSV_FIELDS)
            writer.writeheader()
            for rec in snapshot:
                writer.writerow(rec.to_flat_dict())

        logger.info(f"Position log saved to {path} ({len(snapshot)} records)")

    def save_json(self, filepath: str) -> None:
        """Write all records to a JSON file."""
        path = Path(filepath)
        with self._lock:
            snapshot = list(self._records)

        data = {
            "created": datetime.now().isoformat(),
            "record_count": len(snapshot),
            "records": [asdict(r) for r in snapshot],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        logger.info(f"Position log saved to {path} ({len(snapshot)} records)")

    @classmethod
    def load_json(cls, filepath: str) -> PositionLogger:
        """Load a position log from a previously exported JSON file."""
        instance = cls()
        path = Path(filepath)

        with open(path) as f:
            data = json.load(f)

        for rd in data.get("records", []):
            meta = rd.get("metadata", {})
            if isinstance(meta, str):
                try:
                    meta = json.loads(meta)
                except (json.JSONDecodeError, TypeError):
                    meta = {}

            instance._records.append(PositionRecord(
                timestamp=rd.get("timestamp", ""),
                event=rd.get("event", ""),
                xy_x=rd.get("xy_x"),
                xy_y=rd.get("xy_y"),
                xy_f=rd.get("xy_f"),
                z=rd.get("z"),
                p1=rd.get("p1"),
                p2=rd.get("p2"),
                p3=rd.get("p3"),
                metadata=meta,
            ))

        logger.info(f"Loaded {instance.count} records from {path}")
        return instance

    # ── Utility ───────────────────────────────────────────────────

    @staticmethod
    def generate_filename(prefix: str = "position_log") -> str:
        """Generate a timestamped filename."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{prefix}_{ts}.csv"
