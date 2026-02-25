"""
Position Logger - Record timestamped positions during calibration and printing.

Records positions, events, and metadata for reproducibility and debugging.
Supports export to CSV and JSON formats.

Usage:
    logger = PositionLogger()
    logger.record("calibrate_zero", xy_pos=(1000, 2000, 0), zp_pos=(0.1, 0, 0, 0))
    logger.record("print_start", xy_pos=(1000, 2000, 0), zp_pos=(0.5, 0, 0, 0),
                   metadata={"job_name": "Test Print"})
    logger.save_csv("position_log.csv")
    logger.save_json("position_log.json")
"""

from __future__ import annotations

import csv
import json
import logging
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class PositionRecord:
    """A single timestamped position record."""
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

    def to_flat_dict(self) -> dict:
        """Flatten for CSV export (metadata as JSON string)."""
        d = {
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
        return d


class PositionLogger:
    """
    Records timestamped positions during calibration and printing.
    
    Thread-safe via append-only list (CPython GIL is sufficient here).
    """

    def __init__(self):
        self._records: list[PositionRecord] = []

    @property
    def count(self) -> int:
        """Number of recorded entries."""
        return len(self._records)

    @property
    def records(self) -> list[PositionRecord]:
        """All records (read-only copy)."""
        return list(self._records)

    def record(
        self,
        event: str,
        xy_pos: Optional[tuple] = None,
        zp_pos: Optional[tuple] = None,
        metadata: Optional[dict] = None,
    ):
        """
        Record a position event.
        
        Args:
            event: Event name (e.g., "calibrate_zero", "print_start", "move_xy")
            xy_pos: (x, y, f) tuple from XY stage or None
            zp_pos: (z, p1, p2, p3) tuple from ZP stage or None
            metadata: Optional dict of additional info
        """
        now = datetime.now().isoformat(timespec="milliseconds")

        rec = PositionRecord(
            timestamp=now,
            event=event,
            metadata=metadata or {},
        )

        if xy_pos is not None:
            rec.xy_x = xy_pos[0] if len(xy_pos) > 0 else None
            rec.xy_y = xy_pos[1] if len(xy_pos) > 1 else None
            rec.xy_f = xy_pos[2] if len(xy_pos) > 2 else None

        if zp_pos is not None:
            rec.z = zp_pos[0] if len(zp_pos) > 0 else None
            rec.p1 = zp_pos[1] if len(zp_pos) > 1 else None
            rec.p2 = zp_pos[2] if len(zp_pos) > 2 else None
            rec.p3 = zp_pos[3] if len(zp_pos) > 3 else None

        self._records.append(rec)
        logger.debug(f"Position logged: {event} xy={xy_pos} zp={zp_pos}")

    def clear(self):
        """Clear all records."""
        self._records.clear()
        logger.info("Position log cleared")

    # ── Export ─────────────────────────────────────────────────────

    def save_csv(self, filepath: str):
        """
        Save all records to a CSV file.
        
        Args:
            filepath: Output CSV file path
        """
        path = Path(filepath)
        fieldnames = [
            "timestamp", "event",
            "xy_x", "xy_y", "xy_f",
            "z", "p1", "p2", "p3",
            "metadata",
        ]

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for rec in self._records:
                writer.writerow(rec.to_flat_dict())

        logger.info(f"Position log saved to {path} ({len(self._records)} records)")

    def save_json(self, filepath: str):
        """
        Save all records to a JSON file.
        
        Args:
            filepath: Output JSON file path
        """
        path = Path(filepath)
        data = {
            "created": datetime.now().isoformat(),
            "record_count": len(self._records),
            "records": [asdict(r) for r in self._records],
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        logger.info(f"Position log saved to {path} ({len(self._records)} records)")

    @classmethod
    def load_json(cls, filepath: str) -> "PositionLogger":
        """
        Load a position log from a JSON file.
        
        Args:
            filepath: Input JSON file path
            
        Returns:
            PositionLogger with loaded records
        """
        instance = cls()
        path = Path(filepath)

        with open(path, "r") as f:
            data = json.load(f)

        for rec_data in data.get("records", []):
            meta = rec_data.get("metadata", {})
            if isinstance(meta, str):
                try:
                    meta = json.loads(meta)
                except (json.JSONDecodeError, TypeError):
                    meta = {}

            instance._records.append(PositionRecord(
                timestamp=rec_data.get("timestamp", ""),
                event=rec_data.get("event", ""),
                xy_x=rec_data.get("xy_x"),
                xy_y=rec_data.get("xy_y"),
                xy_f=rec_data.get("xy_f"),
                z=rec_data.get("z"),
                p1=rec_data.get("p1"),
                p2=rec_data.get("p2"),
                p3=rec_data.get("p3"),
                metadata=meta,
            ))

        logger.info(f"Loaded position log from {path} ({instance.count} records)")
        return instance

    def generate_filename(self, prefix: str = "position_log") -> str:
        """Generate a timestamped filename for export."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{prefix}_{ts}.csv"
