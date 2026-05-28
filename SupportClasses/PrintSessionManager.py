"""
PrintSessionManager.py — Unified Print Session persistence for MEBP v7.5.0.

A *Print Session* bundles everything the new wizard-style Print Setup
page needs to fully describe a print run in a single JSON file:

  - metadata        (name, description, author, created, modified)
  - hardware_ref    (by-reference HW config — config_name + hw_version
                     + fingerprint; the HW config itself stays in
                     ``config/hardware/devices/``)
  - workspace       (WorkspaceConfig.to_dict())
  - objects         (PrintObject dicts — trajectory only persisted for
                     csv-sourced objects; parametric regenerate on load)
  - collections     (PrintCollection dicts)
  - layout_presets  (layout pattern presets)
  - wells           (WellSetupModel.to_dict())
  - execution_config (PrintExecutionConfig.to_dict())
  - ui              (current_step, side_panel_collapsed)

Sessions live in ``config/sessions/*.json`` so they sit alongside the
existing print-files and hardware profiles without disturbing either.

The old ``PrintFileManager`` is preserved for read-only legacy import
and continues to be the source-of-truth for the v7.2.3 print-file
shape; ``PrintSession`` is its v7.5.0 successor for end-to-end session
state.
"""

from __future__ import annotations

import copy
import json
import logging
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


SCHEMA_VERSION = "7.5.0"
DEFAULT_SESSIONS_DIR = "config/sessions"


# ═══════════════════════════════════════════════════════════════════
# Dataclasses
# ═══════════════════════════════════════════════════════════════════


@dataclass
class SessionMetadata:
    name: str = "Untitled"
    description: str = ""
    created: str = ""
    modified: str = ""
    author: str = ""

    def __post_init__(self) -> None:
        now = datetime.now(timezone.utc).isoformat()
        if not self.created:
            self.created = now
        if not self.modified:
            self.modified = now

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "created": self.created,
            "modified": self.modified,
            "author": self.author,
        }

    @classmethod
    def from_dict(cls, data: dict) -> SessionMetadata:
        return cls(
            name=data.get("name", "Untitled"),
            description=data.get("description", ""),
            created=data.get("created", ""),
            modified=data.get("modified", ""),
            author=data.get("author", ""),
        )


@dataclass
class HardwareRef:
    """By-reference handle to the HardwareConfig this session was built
    against. The actual HW config lives in ``config/hardware/devices/``;
    we only persist its name + a small fingerprint so the session is
    portable across machines that share a named profile."""
    config_name: str = ""
    hw_version: str = ""
    fingerprint: str = ""    # short stable identifier (e.g. "ME3B-V2")

    def to_dict(self) -> dict:
        return {
            "config_name": self.config_name,
            "hw_version": self.hw_version,
            "fingerprint": self.fingerprint,
        }

    @classmethod
    def from_dict(cls, data: dict) -> HardwareRef:
        return cls(
            config_name=data.get("config_name", ""),
            hw_version=data.get("hw_version", ""),
            fingerprint=data.get("fingerprint", ""),
        )

    @classmethod
    def from_hardware_config(cls, hw_config: Any) -> HardwareRef:
        if hw_config is None:
            return cls()
        return cls(
            config_name=getattr(hw_config, "name", "") or "",
            hw_version=getattr(hw_config, "version", "") or "",
            fingerprint=getattr(hw_config, "fingerprint", "") or "",
        )


@dataclass
class PrintSession:
    """Complete in-memory session state."""
    schema_version: str = SCHEMA_VERSION
    metadata: SessionMetadata = field(default_factory=SessionMetadata)
    hardware_ref: HardwareRef = field(default_factory=HardwareRef)

    # Step state — each piece is a serializable dict produced by the
    # owning model's to_dict(). Stored as opaque dicts here so the
    # session manager doesn't take a dependency on UI / model code.
    workspace: dict = field(default_factory=dict)
    objects: list[dict] = field(default_factory=list)
    collections: list[dict] = field(default_factory=list)
    layout_presets: dict = field(default_factory=dict)
    wells: dict = field(default_factory=dict)
    execution_config: dict = field(default_factory=dict)
    ui: dict = field(default_factory=dict)

    _dirty: bool = field(default=False, repr=False)

    @property
    def name(self) -> str:
        return self.metadata.name

    @name.setter
    def name(self, value: str) -> None:
        self.metadata.name = value
        self.mark_dirty()

    @property
    def is_dirty(self) -> bool:
        return self._dirty

    def mark_dirty(self) -> None:
        self._dirty = True
        self.metadata.modified = datetime.now(timezone.utc).isoformat()

    def mark_clean(self) -> None:
        self._dirty = False

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "metadata": self.metadata.to_dict(),
            "hardware_ref": self.hardware_ref.to_dict(),
            "workspace": copy.deepcopy(self.workspace),
            "objects": copy.deepcopy(self.objects),
            "collections": copy.deepcopy(self.collections),
            "layout_presets": copy.deepcopy(self.layout_presets),
            "wells": copy.deepcopy(self.wells),
            "execution_config": copy.deepcopy(self.execution_config),
            "ui": copy.deepcopy(self.ui),
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintSession:
        s = cls()
        s.schema_version = data.get("schema_version", SCHEMA_VERSION)
        if "metadata" in data:
            s.metadata = SessionMetadata.from_dict(data["metadata"])
        if "hardware_ref" in data:
            s.hardware_ref = HardwareRef.from_dict(data["hardware_ref"])
        s.workspace = data.get("workspace", {})
        s.objects = data.get("objects", [])
        s.collections = data.get("collections", [])
        s.layout_presets = data.get("layout_presets", {})
        s.wells = data.get("wells", {})
        s.execution_config = data.get("execution_config", {})
        s.ui = data.get("ui", {})
        s._dirty = False
        return s

    def validate_collection_refs(self) -> list[str]:
        """Return a list of well names whose ``print_collections``
        reference a collection name that doesn't exist in this session.
        Empty list means everything resolves."""
        collection_names = {c.get("name") for c in self.collections if c.get("name")}
        assignments = (self.wells or {}).get("assignments", {})
        missing: list[str] = []
        for well_name, wa in assignments.items():
            refs = wa.get("print_collections", []) if isinstance(wa, dict) else []
            for ref in refs:
                if ref not in collection_names:
                    missing.append(well_name)
                    break
        return missing


# ═══════════════════════════════════════════════════════════════════
# Filename helpers
# ═══════════════════════════════════════════════════════════════════


SESSION_SUFFIX = ".print.json"


def _sanitize_stem(name: str) -> str:
    """Return a safe filename stem (no extension) for ``name``."""
    if not name or not name.strip():
        return "Untitled"
    safe = re.sub(r"[^\w\-]", "_", name.strip())
    safe = re.sub(r"_+", "_", safe).strip("_")
    return safe[:60] or "Untitled"


# ═══════════════════════════════════════════════════════════════════
# Trajectory Regeneration
# ═══════════════════════════════════════════════════════════════════


def regenerate_parametric_trajectories(
    objects: list[dict],
    hw_config: Any,
) -> int:
    """
    For each parametric object dict (i.e. ``source != "csv"``), populate
    its ``trajectory`` field by running ``generate_object_trajectory()``
    with the live ``hw_config``'s needle + syringe map.

    Csv-sourced objects already carry their persisted trajectory and are
    left alone.

    Returns the number of objects regenerated. Logs and skips individual
    failures so one bad object doesn't break a whole session load.
    """
    if not objects or hw_config is None:
        return 0

    try:
        from SupportClasses.GeometryEngine import (
            PrintObject, generate_object_trajectory,
        )
    except ImportError as e:
        logger.warning(f"GeometryEngine unavailable; skipping regen: {e}")
        return 0

    needle = getattr(hw_config, "needle", None)
    if needle is None:
        logger.warning("HW config has no needle; cannot regenerate trajectories")
        return 0

    # Build {pump_id: syringe} map from hw_config.pumps
    syringe_map: dict[str, Any] = {}
    pumps = getattr(hw_config, "pumps", {}) or {}
    for pump_id, pcfg in pumps.items():
        syringe = getattr(pcfg, "syringe", None)
        if syringe is not None:
            syringe_map[pump_id] = syringe

    regenerated = 0
    for obj_dict in objects:
        if not isinstance(obj_dict, dict):
            continue
        if obj_dict.get("source") == "csv":
            continue
        if "trajectory" in obj_dict and obj_dict["trajectory"]:
            # Already populated (e.g. session was just saved in-memory)
            continue
        try:
            obj = PrintObject.from_dict(copy.deepcopy(obj_dict))
            # Pick first pump from ink_assignments, or fall back to P1
            pump_id = next(iter(obj.ink_assignments.keys()), "P1")
            generate_object_trajectory(
                obj, needle, syringe_map, pump_id=pump_id,
            )
            if obj.has_trajectory:
                obj_dict["trajectory"] = obj.trajectory.tolist()
                obj_dict["total_length_mm"] = obj.total_length_mm
                obj_dict["total_volume_uL"] = obj.total_volume_uL
                obj_dict["total_time_s"] = obj.total_time_s
                obj_dict["num_layers"] = obj.num_layers
                regenerated += 1
        except Exception as e:
            logger.warning(
                f"Failed to regenerate trajectory for "
                f"'{obj_dict.get('name', '?')}': {e}"
            )
    return regenerated


# ═══════════════════════════════════════════════════════════════════
# Manager
# ═══════════════════════════════════════════════════════════════════


class PrintSessionManager:
    """Filesystem-backed CRUD manager for Print Sessions.

    Sessions are JSON files in ``config/sessions/``. The manager keeps
    no in-memory "current session" (unlike PrintFileManager) — callers
    pass a PrintSession instance to ``save_session`` directly. This
    keeps the manager stateless and easier to use from the wizard
    orchestrator.
    """

    def __init__(self, sessions_dir: str | Path = DEFAULT_SESSIONS_DIR):
        self.sessions_dir = Path(sessions_dir)
        self.sessions_dir.mkdir(parents=True, exist_ok=True)

    # ── List ──────────────────────────────────────────────────────

    def list_sessions(self) -> list[dict]:
        """List all session files with metadata summaries.

        Returns dicts: {name, path, modified, hardware_ref, well_count}.
        """
        results: list[dict] = []
        for fp in sorted(self.sessions_dir.glob("*" + SESSION_SUFFIX)):
            try:
                with open(fp) as f:
                    data = json.load(f)
                meta = data.get("metadata", {})
                hwr = data.get("hardware_ref", {})
                wells = data.get("wells", {})
                results.append({
                    "name": meta.get("name", fp.stem),
                    "path": str(fp),
                    "modified": meta.get("modified", ""),
                    "description": meta.get("description", ""),
                    "hardware_ref": hwr.get("config_name", ""),
                    "well_count": len(wells.get("assignments", {})
                                      if isinstance(wells, dict) else {}),
                })
            except (json.JSONDecodeError, OSError) as e:
                logger.warning(f"Skipping malformed session file {fp}: {e}")
        return results

    # ── New ───────────────────────────────────────────────────────

    def new_session(
        self,
        name: str = "Untitled",
        hw_config: Any = None,
    ) -> PrintSession:
        """Create a blank PrintSession (not yet persisted)."""
        existing = {info["name"] for info in self.list_sessions()}
        base_name = name
        counter = 1
        while name in existing:
            counter += 1
            name = f"{base_name}_{counter}"

        session = PrintSession(
            metadata=SessionMetadata(name=name),
            hardware_ref=HardwareRef.from_hardware_config(hw_config),
        )
        return session

    # ── Save ──────────────────────────────────────────────────────

    def save_session(
        self,
        session: PrintSession,
        path: Path | str | None = None,
    ) -> Path | None:
        """Persist ``session`` to disk. If ``path`` is None, derives the
        path from the session name. Returns the saved path, or None on
        failure."""
        if path is None:
            path = self.sessions_dir / (_sanitize_stem(session.name) + SESSION_SUFFIX)
        path = Path(path)
        if not path.name.endswith(SESSION_SUFFIX):
            path = path.with_name(path.stem + SESSION_SUFFIX)

        try:
            with open(path, "w") as f:
                json.dump(session.to_dict(), f, indent=2)
            session.mark_clean()
            logger.info(f"Saved session: {session.name} → {path}")
            return path
        except OSError as e:
            logger.error(f"Failed to save session at {path}: {e}")
            return None

    # ── Load ──────────────────────────────────────────────────────

    def load_session(
        self,
        path: Path | str,
        hw_config: Any = None,
        regenerate_trajectories: bool = True,
        strict_hw_match: bool = False,
    ) -> PrintSession | None:
        """Load a session from disk.

        Args:
            path: file path to load
            hw_config: current live HardwareConfig (needed if
                ``regenerate_trajectories`` is True)
            regenerate_trajectories: if True (default), parametric
                objects have their trajectories rebuilt from the live
                hw_config's needle + syringe map
            strict_hw_match: if True, refuse to load when the session's
                ``hardware_ref.config_name`` does not match
                ``hw_config.name``. Default False (permissive).
        """
        path = Path(path)
        if not path.exists():
            logger.warning(f"Session file not found: {path}")
            return None

        try:
            with open(path) as f:
                raw = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            logger.error(f"Failed to read session at {path}: {e}")
            return None

        session = PrintSession.from_dict(raw)

        if strict_hw_match and hw_config is not None:
            current_name = getattr(hw_config, "name", "") or ""
            if (session.hardware_ref.config_name
                    and session.hardware_ref.config_name != current_name):
                logger.error(
                    f"Session was built against HW '{session.hardware_ref.config_name}'"
                    f" but current is '{current_name}' (strict_hw_match=True)"
                )
                return None

        if regenerate_trajectories and hw_config is not None:
            n = regenerate_parametric_trajectories(session.objects, hw_config)
            if n:
                logger.info(f"Regenerated {n} parametric trajectories on load")

        logger.info(f"Loaded session: {session.name} from {path}")
        return session

    # ── Delete ────────────────────────────────────────────────────

    def delete_session(self, name: str) -> bool:
        """Delete a session by its display name."""
        for fp in self.sessions_dir.glob("*" + SESSION_SUFFIX):
            try:
                with open(fp) as f:
                    data = json.load(f)
                if data.get("metadata", {}).get("name") == name:
                    fp.unlink()
                    logger.info(f"Deleted session: {name}")
                    return True
            except (json.JSONDecodeError, OSError):
                continue

        # Fallback: try sanitized filename
        candidate = self.sessions_dir / (_sanitize_stem(name) + SESSION_SUFFIX)
        if candidate.exists():
            candidate.unlink()
            logger.info(f"Deleted session: {name}")
            return True

        logger.warning(f"Session not found for deletion: {name}")
        return False

    # ── Duplicate ─────────────────────────────────────────────────

    def duplicate_session(
        self,
        name: str,
        new_name: str,
    ) -> PrintSession | None:
        """Deep-copy an existing session under a new name and persist
        the copy."""
        source_path: Path | None = None
        for fp in self.sessions_dir.glob("*" + SESSION_SUFFIX):
            try:
                with open(fp) as f:
                    data = json.load(f)
                if data.get("metadata", {}).get("name") == name:
                    source_path = fp
                    break
            except (json.JSONDecodeError, OSError):
                continue

        if source_path is None:
            logger.warning(f"Source session not found: {name}")
            return None

        try:
            with open(source_path) as f:
                raw = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            logger.error(f"Failed to read source session at {source_path}: {e}")
            return None

        dup = PrintSession.from_dict(raw)
        dup.metadata.name = new_name
        now = datetime.now(timezone.utc).isoformat()
        dup.metadata.created = now
        dup.metadata.modified = now

        if self.save_session(dup) is None:
            return None
        return dup
