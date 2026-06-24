"""WorkflowSettingsStore.py — per-workflow, saveable/loadable settings profiles.

v7.5.x: Each workflow (Spheroid Pick & Place, Cell Targeting & Removal, Quick
Print, ZP Stress Test, XY↔ZP Timing Calibration) has a comprehensive settings
popout. The operator can tune those settings, **save them to a named file for
that workflow, and reload them later** — so a user keeps the settings they like
and can switch between presets.

This store owns that persistence. Each workflow gets its own directory:

    config/workflows/<workflow_id>/<Profile Name>.json   ← named, user-managed
    config/workflows/<workflow_id>/__last__.json          ← auto-saved last values

A settings file is a flat ``values`` dict (key → JSON scalar / small combo
token) wrapped with a tiny header so a file can be recognised + shared:

    {
      "_format": "mebp-workflow-settings",
      "workflow_id": "spheroid_pickup",
      "name": "Big spheroids slow",
      "saved": "2026-06-22T10:15:00",
      "values": { "diameter": 300.0, "pickup_speed_uL_s": 0.4, ... }
    }

All writes are atomic (tmp + ``os.replace``) so a crash can't truncate a file —
the same convention the calibration stores use.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

_FORMAT = "mebp-workflow-settings"

# Base directory for all per-workflow settings. Resolved relative to the repo
# root (this file lives in SupportClasses/, so two parents up is the root).
_BASE_DIR = Path(__file__).resolve().parent.parent / "config" / "workflows"

# The reserved stem for the auto-saved "current values" file (hidden from the
# named-profile list the operator manages).
_LAST_STEM = "__last__"


def _sanitize(name: str) -> str:
    """Make a user profile name safe to use as a filename stem."""
    safe = (name or "").strip() or "Untitled"
    for ch in '/\\:*?"<>|':
        safe = safe.replace(ch, "_")
    return safe


class WorkflowSettingsStore:
    """Load/save/list named settings profiles for one workflow."""

    def __init__(self, workflow_id: str, base_dir: Path | str | None = None):
        self.workflow_id = str(workflow_id)
        if base_dir is not None:
            base = Path(base_dir)
        else:
            # An env override lets tests / CI isolate the settings directory so
            # automated runs never read or write the repo's config/workflows.
            env = os.environ.get("MEBP_WORKFLOW_SETTINGS_DIR")
            base = Path(env) if env else _BASE_DIR
        self.dir = base / self.workflow_id

    # ── Paths ─────────────────────────────────────────────────────

    def profile_path(self, name: str) -> Path:
        stem = _sanitize(name)
        # Never let a user profile collide with the reserved auto-saved file.
        if stem == _LAST_STEM:
            stem = stem + "_"
        return self.dir / f"{stem}.json"

    def _last_path(self) -> Path:
        return self.dir / f"{_LAST_STEM}.json"

    # ── Listing ───────────────────────────────────────────────────

    def list_profiles(self) -> list[str]:
        """Return saved profile display names (sorted), excluding ``__last__``."""
        if not self.dir.is_dir():
            return []
        names: list[str] = []
        for jf in sorted(self.dir.glob("*.json")):
            if jf.stem == _LAST_STEM:
                continue
            try:
                data = json.loads(jf.read_text(encoding="utf-8"))
                names.append(str(data.get("name") or jf.stem))
            except Exception as exc:  # malformed file — skip, don't crash
                logger.debug("Skipping malformed workflow-settings %s: %s",
                             jf.name, exc)
        # De-dup while preserving sort order.
        seen: set[str] = set()
        out: list[str] = []
        for n in names:
            if n not in seen:
                seen.add(n)
                out.append(n)
        return out

    # ── Read ──────────────────────────────────────────────────────

    @staticmethod
    def _values_of(data: object) -> Optional[dict]:
        """Extract the ``values`` dict from a parsed settings file.

        Accepts the wrapped format (``{"values": {...}}``) and, defensively, a
        bare values dict (so a hand-edited file still loads)."""
        if not isinstance(data, dict):
            return None
        vals = data.get("values")
        if isinstance(vals, dict):
            return dict(vals)
        # Bare dict that isn't our wrapper — treat the whole thing as values
        # unless it's clearly a header-only file.
        if "_format" not in data and "values" not in data:
            return dict(data)
        return None

    def load_profile(self, name: str) -> Optional[dict]:
        p = self.profile_path(name)
        return self._read(p)

    def load_last(self) -> Optional[dict]:
        return self._read(self._last_path())

    def read_file(self, path: Path | str) -> Optional[dict]:
        """Read an arbitrary settings file the user picked (Import…)."""
        return self._read(Path(path))

    def _read(self, path: Path) -> Optional[dict]:
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            logger.warning("WorkflowSettingsStore: failed to read %s: %s",
                           path, exc)
            return None
        return self._values_of(data)

    # ── Write ─────────────────────────────────────────────────────

    def save_profile(self, name: str, values: dict) -> Path:
        path = self.profile_path(name)
        self._write(path, values, name=name)
        return path

    def save_last(self, values: dict) -> Path:
        path = self._last_path()
        self._write(path, values, name=_LAST_STEM)
        return path

    def write_file(self, path: Path | str, values: dict,
                   name: str | None = None) -> Path:
        """Write to an arbitrary path the user picked (Export…)."""
        p = Path(path)
        self._write(p, values, name=name or p.stem)
        return p

    def _write(self, path: Path, values: dict, *, name: str) -> None:
        """Atomic write. RAISES on failure so an explicit Save/Save As/Export
        can report it to the operator (silent failure = perceived data loss).
        ``save_last`` (best-effort, on hide) wraps this in its own try/except."""
        payload = {
            "_format": _FORMAT,
            "workflow_id": self.workflow_id,
            "name": name,
            "saved": datetime.now().isoformat(timespec="seconds"),
            "values": dict(values or {}),
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            os.replace(tmp, path)
        finally:
            if os.path.exists(tmp):
                try:
                    os.remove(tmp)
                except OSError:
                    pass
        logger.debug("WorkflowSettingsStore: saved %s", path)

    # ── Delete ────────────────────────────────────────────────────

    def delete_profile(self, name: str) -> bool:
        p = self.profile_path(name)
        try:
            if p.exists():
                p.unlink()
                logger.info("Workflow-settings profile deleted: %s", p)
            return True
        except Exception as exc:
            logger.warning("Failed to delete workflow-settings %s: %s", p, exc)
            return False
