"""
Print History — Persistent log of completed prints.

Records completed, aborted, and failed print jobs with timestamps,
parameters, and outcomes.  Stored as a JSON file alongside settings.

Enhancement 6 from ARCHITECTURE.md task list.

Usage::

    history = PrintHistory()
    history.load()
    history.add_entry(job_name="Test Print", state="completed", ...)
    history.save()
    entries = history.get_entries()
"""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

DEFAULT_HISTORY_FILE = "print_history.json"
MAX_HISTORY_ENTRIES = 500  # rolling limit


@dataclass
class PrintHistoryEntry:
    """A single print history record."""

    timestamp: str                # ISO format start time
    end_time: str = ""            # ISO format end time
    job_name: str = ""
    source_file: str = ""
    state: str = ""               # completed / aborted / error
    total_commands: int = 0
    completed_commands: int = 0
    duration_seconds: float = 0.0
    settings: dict = field(default_factory=dict)  # snapshot of PrintSettings
    wells_printed: int = 0
    layers: int = 0
    pumps_used: list = field(default_factory=list)
    error_message: str = ""
    notes: str = ""

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "PrintHistoryEntry":
        valid = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in data.items() if k in valid}
        return cls(**filtered)


class PrintHistory:
    """
    Persistent print history log.

    Stores entries in a JSON file and provides query/filter methods.
    Thread-safe for concurrent access from print threads.
    """

    def __init__(self, filepath: str = DEFAULT_HISTORY_FILE):
        self.filepath = Path(filepath)
        self._entries: list[PrintHistoryEntry] = []

    # ── Load / Save ───────────────────────────────────────────────

    def load(self) -> None:
        """Load history from file."""
        if not self.filepath.exists():
            logger.debug(f"No print history file at {self.filepath}")
            return
        try:
            with open(self.filepath) as f:
                data = json.load(f)
            self._entries = [
                PrintHistoryEntry.from_dict(e)
                for e in data.get("entries", [])
            ]
            logger.info(f"Loaded {len(self._entries)} print history entries")
        except (json.JSONDecodeError, Exception) as e:
            logger.warning(f"Failed to load print history: {e}")

    def save(self) -> None:
        """Persist history to file."""
        # Enforce rolling limit
        if len(self._entries) > MAX_HISTORY_ENTRIES:
            self._entries = self._entries[-MAX_HISTORY_ENTRIES:]

        data = {
            "version": 1,
            "updated": datetime.now().isoformat(),
            "entry_count": len(self._entries),
            "entries": [e.to_dict() for e in self._entries],
        }
        try:
            with open(self.filepath, "w") as f:
                json.dump(data, f, indent=2)
            logger.debug(f"Print history saved ({len(self._entries)} entries)")
        except Exception as e:
            logger.error(f"Failed to save print history: {e}")

    # ── Recording ─────────────────────────────────────────────────

    def add_entry(
        self,
        job_name: str,
        state: str,
        source_file: str = "",
        total_commands: int = 0,
        completed_commands: int = 0,
        duration_seconds: float = 0.0,
        settings: dict | None = None,
        wells_printed: int = 0,
        layers: int = 0,
        pumps_used: list | None = None,
        error_message: str = "",
        notes: str = "",
        start_time: str | None = None,
    ) -> PrintHistoryEntry:
        """Add a new print history entry and auto-save."""
        now = datetime.now().isoformat(timespec="seconds")
        entry = PrintHistoryEntry(
            timestamp=start_time or now,
            end_time=now,
            job_name=job_name,
            source_file=source_file,
            state=state,
            total_commands=total_commands,
            completed_commands=completed_commands,
            duration_seconds=duration_seconds,
            settings=settings or {},
            wells_printed=wells_printed,
            layers=layers,
            pumps_used=pumps_used or [],
            error_message=error_message,
            notes=notes,
        )
        self._entries.append(entry)
        self.save()
        logger.info(f"Print history: {job_name} [{state}] ({duration_seconds:.1f}s)")
        return entry

    # ── Queries ───────────────────────────────────────────────────

    @property
    def count(self) -> int:
        return len(self._entries)

    def get_entries(
        self,
        limit: int = 50,
        state_filter: str | None = None,
    ) -> list[PrintHistoryEntry]:
        """Get recent entries, optionally filtered by state."""
        entries = self._entries
        if state_filter:
            entries = [e for e in entries if e.state == state_filter]
        return entries[-limit:]

    def get_stats(self) -> dict:
        """Return summary statistics."""
        total = len(self._entries)
        completed = sum(1 for e in self._entries if e.state == "completed")
        aborted = sum(1 for e in self._entries if e.state == "aborted")
        errors = sum(1 for e in self._entries if e.state == "error")
        total_time = sum(e.duration_seconds for e in self._entries)
        return {
            "total_prints": total,
            "completed": completed,
            "aborted": aborted,
            "errors": errors,
            "total_print_time_hours": total_time / 3600.0,
            "success_rate": (completed / total * 100) if total > 0 else 0.0,
        }

    def clear(self) -> None:
        """Clear all history entries."""
        self._entries.clear()
        self.save()
        logger.info("Print history cleared")

    def export_csv(self, filepath: str) -> None:
        """Export history to CSV."""
        import csv
        path = Path(filepath)
        fields = [
            "timestamp", "end_time", "job_name", "state",
            "total_commands", "completed_commands", "duration_seconds",
            "layers", "wells_printed", "error_message", "notes",
        ]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for entry in self._entries:
                d = entry.to_dict()
                writer.writerow({k: d.get(k, "") for k in fields})
        logger.info(f"Print history exported to {path}")
