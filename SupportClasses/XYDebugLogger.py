"""
XY Stage Debug Logger — correlates raw serial position data with jog commands.

Enabled at startup with:  python main.py --debug-xy
Or at runtime via:        from SupportClasses.XYDebugLogger import enable; enable()

Output: timestamped CSV in project root, e.g. xy_debug_20260313_142131.csv

Columns
-------
t_ms        : milliseconds since logging started
event       : POLL | JOG_CMD | MOVE_REL | MOVE_VEL | QUERY_ERR | PARSE_ERR
raw_rx      : raw serial response string (position queries only)
pos_x       : parsed stage X coordinate (µm)
pos_y       : parsed stage Y coordinate (µm)
pos_z       : parsed stage Z coordinate (µm)
cmd_dx      : requested X delta (µm) — JOG_CMD
cmd_dy      : requested Y delta (µm) — JOG_CMD
sent_dx     : actual sent X delta after safety clamping (µm) — MOVE_REL
sent_dy     : actual sent Y delta after safety clamping (µm) — MOVE_REL
vx          : X velocity — MOVE_VEL
vy          : Y velocity — MOVE_VEL
cached_x    : cached stage X at jog command time (µm) — JOG_CMD
cached_y    : cached stage Y at jog command time (µm) — JOG_CMD
note        : extra context (e.g. "safety clamped", error message)
"""
from __future__ import annotations

import csv
import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

# ── module-level state ────────────────────────────────────────────────────────
_lock = threading.Lock()
_file = None
_writer: csv.DictWriter | None = None
_enabled: bool = False
_t0: float = 0.0

FIELDS = [
    "t_ms", "event",
    "raw_rx",
    "pos_x", "pos_y", "pos_z",
    "cmd_dx", "cmd_dy",
    "sent_dx", "sent_dy",
    "vx", "vy",
    "cached_x", "cached_y",
    "note",
]


# ── public API ────────────────────────────────────────────────────────────────

def enable(filepath: str | None = None) -> str:
    """Start capturing to *filepath* (auto-named if None). Returns the path."""
    global _file, _writer, _enabled, _t0
    _t0 = time.monotonic()
    if filepath is None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        filepath = str(Path(__file__).parent.parent / f"xy_debug_{ts}.csv")
    with _lock:
        _file = open(filepath, "w", newline="", encoding="utf-8")
        _writer = csv.DictWriter(_file, fieldnames=FIELDS, extrasaction="ignore")
        _writer.writeheader()
        _file.flush()
        _enabled = True
    logger.info(f"[XYDebug] Logging to: {filepath}")
    return filepath


def disable() -> None:
    """Stop capturing and close the CSV file."""
    global _enabled, _writer, _file
    with _lock:
        _enabled = False
        if _file is not None:
            _file.close()
            _file = None
        _writer = None
    logger.info("[XYDebug] Debug logging disabled")


def is_enabled() -> bool:
    return _enabled


def log(event: str, **fields) -> None:
    """Write one row. Silently no-ops when disabled."""
    if not _enabled:
        return
    row = {k: "" for k in FIELDS}
    row["t_ms"] = f"{(time.monotonic() - _t0) * 1000:.1f}"
    row["event"] = event
    row.update({k: v for k, v in fields.items() if k in FIELDS})
    with _lock:
        if _writer is not None:
            _writer.writerow(row)
            _file.flush()
