"""
PrintFileManager.py — Persistent Print File CRUD for MEBP v7.2.3.

Manages print files stored as JSON in ``config/prints/``. Each print file
contains objects, collections, layout presets, and metadata.

New for v7.2.3:
    - File-centric workflow (design → save → assign to wells)
    - Auto-save with dirty tracking
    - Schema validation and forward migration
    - Layout preset storage

Usage::

    mgr = PrintFileManager(prints_dir="config/prints")
    mgr.new_file("My Scaffold")
    mgr.add_object("Ring_1", {"object_type": "circle", "params": {...}, ...})
    mgr.save()
    names = mgr.list_files()
    mgr.load("My Scaffold")
"""

from __future__ import annotations

import copy
import json
import logging
import re
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

SCHEMA_VERSION = "7.5.0"
DEFAULT_PRINTS_DIR = "config/prints"


# ═══════════════════════════════════════════════════════════════════
# Data Classes
# ═══════════════════════════════════════════════════════════════════

@dataclass
class PrintFileMetadata:
    """Metadata for a print file."""
    name: str = "Untitled"
    description: str = ""
    created: str = ""
    modified: str = ""
    author: str = ""

    def __post_init__(self):
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
    def from_dict(cls, data: dict) -> PrintFileMetadata:
        return cls(
            name=data.get("name", "Untitled"),
            description=data.get("description", ""),
            created=data.get("created", ""),
            modified=data.get("modified", ""),
            author=data.get("author", ""),
        )


@dataclass
class PrintFileData:
    """Complete in-memory representation of a print file."""
    schema_version: str = SCHEMA_VERSION
    metadata: PrintFileMetadata = field(default_factory=PrintFileMetadata)
    objects: dict[str, dict] = field(default_factory=dict)
    collections: dict[str, list[dict]] = field(default_factory=dict)
    layout_presets: dict[str, dict] = field(default_factory=dict)
    _dirty: bool = field(default=False, repr=False)

    @property
    def name(self) -> str:
        return self.metadata.name

    @name.setter
    def name(self, value: str):
        self.metadata.name = value
        self.mark_dirty()

    @property
    def is_dirty(self) -> bool:
        return self._dirty

    def mark_dirty(self):
        self._dirty = True
        self.metadata.modified = datetime.now(timezone.utc).isoformat()

    def mark_clean(self):
        self._dirty = False

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "metadata": self.metadata.to_dict(),
            "objects": copy.deepcopy(self.objects),
            "collections": copy.deepcopy(self.collections),
            "layout_presets": copy.deepcopy(self.layout_presets),
        }

    @classmethod
    def from_dict(cls, data: dict) -> PrintFileData:
        pf = cls()
        pf.schema_version = data.get("schema_version", SCHEMA_VERSION)
        if "metadata" in data:
            pf.metadata = PrintFileMetadata.from_dict(data["metadata"])
        pf.objects = data.get("objects", {})
        pf.collections = data.get("collections", {})
        pf.layout_presets = data.get("layout_presets", {})
        pf._dirty = False
        return pf


# ═══════════════════════════════════════════════════════════════════
# Validation
# ═══════════════════════════════════════════════════════════════════

def validate_print_file(data: dict) -> tuple[bool, list[str]]:
    """
    Validate a print file dict against schema v7.2.3.

    Returns:
        (is_valid, list_of_error_messages)
    """
    errors: list[str] = []

    if not isinstance(data, dict):
        return False, ["Root must be a dict"]

    # Schema version
    sv = data.get("schema_version")
    if not sv:
        errors.append("Missing 'schema_version'")

    # Metadata
    meta = data.get("metadata")
    if not isinstance(meta, dict):
        errors.append("Missing or invalid 'metadata' dict")
    else:
        if not meta.get("name"):
            errors.append("metadata.name is required")

    # Objects
    objects = data.get("objects")
    if objects is not None:
        if not isinstance(objects, dict):
            errors.append("'objects' must be a dict")
        else:
            for obj_name, obj_data in objects.items():
                if not isinstance(obj_data, dict):
                    errors.append(f"Object '{obj_name}' must be a dict")
                    continue
                if "object_type" not in obj_data:
                    errors.append(f"Object '{obj_name}' missing 'object_type'")
                if "params" not in obj_data:
                    errors.append(f"Object '{obj_name}' missing 'params'")

    # Collections
    collections = data.get("collections")
    if collections is not None:
        if not isinstance(collections, dict):
            errors.append("'collections' must be a dict")
        else:
            for coll_name, coll_items in collections.items():
                if not isinstance(coll_items, list):
                    errors.append(f"Collection '{coll_name}' must be a list")

    # Layout presets
    presets = data.get("layout_presets")
    if presets is not None and not isinstance(presets, dict):
        errors.append("'layout_presets' must be a dict")

    return len(errors) == 0, errors


# ═══════════════════════════════════════════════════════════════════
# Migration
# ═══════════════════════════════════════════════════════════════════

def migrate_print_file(data: dict) -> dict:
    """
    Forward-migrate a print file to the current schema (v7.5.0).

    Handles:
        - v7.1 → v7.2:   Adds layout_presets if missing
        - v7.2 → v7.2.3: Updates schema_version, ensures collections key
        - v7.2.3 → v7.5.0:
            * Tags each object with ``source`` ("csv" if object_type ==
              "csv_import", else "parametric").
            * Strips the persisted ``trajectory`` array from parametric
              objects — they regenerate via GeometryEngine at load.
            * If an embedded execution_config is present, maps the
              deprecated use_waste/wash/buffer flags into ink_swap.*
              and renames max_ink_volume_uL → pump_volume_overrides_uL.
    """
    version = data.get("schema_version", data.get("version", "7.1"))
    migrated = copy.deepcopy(data)

    # v7.1 → v7.2
    if version < "7.2":
        logger.info(f"Migrating print file from {version} to 7.2")
        migrated.setdefault("layout_presets", {})
        migrated.setdefault("metadata", {
            "name": migrated.pop("print_name", "Migrated"),
            "description": migrated.pop("description", ""),
            "created": migrated.pop("created", ""),
            "modified": migrated.pop("modified", ""),
            "author": "",
        })
        # Convert flat object list to dict if needed
        if isinstance(migrated.get("objects"), list):
            obj_dict = {}
            for i, obj in enumerate(migrated["objects"]):
                name = obj.get("name", f"Object_{i+1}")
                obj_dict[name] = obj
            migrated["objects"] = obj_dict
        migrated["schema_version"] = "7.2"
        version = "7.2"

    # v7.2 → v7.2.3
    if version < "7.2.3":
        logger.info(f"Migrating print file from {version} to 7.2.3")
        migrated.setdefault("layout_presets", {})
        migrated.setdefault("collections", {})
        migrated["schema_version"] = "7.2.3"
        version = "7.2.3"

    # v7.2.3 → v7.5.0
    if version < "7.5.0":
        logger.info(f"Migrating print file from {version} to 7.5.0")
        objects = migrated.get("objects")
        if isinstance(objects, dict):
            for obj_name, obj_data in objects.items():
                if not isinstance(obj_data, dict):
                    continue
                is_csv = (obj_data.get("object_type") == "csv_import"
                          or obj_data.get("source") == "csv")
                obj_data["source"] = "csv" if is_csv else "parametric"
                # Drop persisted trajectory for parametric — it will be
                # regenerated from object_type + params on load.
                if not is_csv:
                    obj_data.pop("trajectory", None)
        # If the file carries an embedded execution_config (newer
        # sessions), rationalize the deprecated flags.
        ec = migrated.get("execution_config")
        if isinstance(ec, dict):
            ink_swap = ec.setdefault("ink_swap", {})
            if "use_waste" in ec:
                ink_swap.setdefault("waste", ec.pop("use_waste"))
            if "use_wash" in ec:
                v = ec.pop("use_wash")
                ink_swap.setdefault("wash_pre", v)
                ink_swap.setdefault("wash_post", v)
            if "use_buffer" in ec:
                ink_swap.setdefault("buffer", ec.pop("use_buffer"))
            if "max_ink_volume_uL" in ec and "pump_volume_overrides_uL" not in ec:
                ec["pump_volume_overrides_uL"] = ec.pop("max_ink_volume_uL")
        migrated["schema_version"] = SCHEMA_VERSION

    # Remove legacy keys
    for key in ("version", "print_name"):
        migrated.pop(key, None)

    return migrated


def write_migration_backup(path: Path, original_data: dict) -> Path | None:
    """
    Write a one-time ``.bak-v7.2.3`` snapshot next to ``path`` containing
    the pre-migration JSON. Returns the backup path, or None if the
    backup already exists (so we never overwrite an existing snapshot).
    """
    backup = path.with_suffix(path.suffix + ".bak-v7.2.3")
    if backup.exists():
        return None
    try:
        with open(backup, "w") as f:
            json.dump(original_data, f, indent=2)
        logger.info(f"Wrote pre-v7.5.0 migration snapshot: {backup}")
        return backup
    except OSError as e:
        logger.warning(f"Failed to write migration backup at {backup}: {e}")
        return None


# ═══════════════════════════════════════════════════════════════════
# Filename Sanitizer
# ═══════════════════════════════════════════════════════════════════

def _sanitize_filename(name: str) -> str:
    """
    Convert a display name to a safe filename (no extension).

    Rules:
        - Replace whitespace/special chars with underscores
        - Collapse consecutive underscores
        - Truncate to 60 chars
        - Fallback to 'Untitled' if empty
    """
    if not name or not name.strip():
        return "Untitled"
    safe = re.sub(r'[^\w\-]', '_', name.strip())
    safe = re.sub(r'_+', '_', safe).strip('_')
    return safe[:60] or "Untitled"


# ═══════════════════════════════════════════════════════════════════
# Print File Manager
# ═══════════════════════════════════════════════════════════════════

class PrintFileManager:
    """
    Filesystem-backed CRUD manager for print files.

    Operates on JSON files in ``config/prints/``.

    Attributes:
        prints_dir: Path to the prints directory
        current: Currently loaded PrintFileData (or None)
        current_path: Path to the file on disk for the current data
    """

    def __init__(self, prints_dir: str | Path = DEFAULT_PRINTS_DIR):
        self.prints_dir = Path(prints_dir)
        self.prints_dir.mkdir(parents=True, exist_ok=True)
        self.current: PrintFileData | None = None
        self.current_path: Path | None = None
        self._last_save_time: float = 0.0

    # ── List ──────────────────────────────────────────────────────

    def list_files(self) -> list[dict]:
        """
        List all print files with metadata summaries.

        Returns list of dicts: {name, path, modified, object_count}
        """
        results = []
        for fp in sorted(self.prints_dir.glob("*.json")):
            try:
                with open(fp) as f:
                    data = json.load(f)
                meta = data.get("metadata", {})
                results.append({
                    "name": meta.get("name", fp.stem),
                    "path": str(fp),
                    "modified": meta.get("modified", ""),
                    "object_count": len(data.get("objects", {})),
                    "description": meta.get("description", ""),
                })
            except (json.JSONDecodeError, OSError) as e:
                logger.warning(f"Skipping malformed print file {fp}: {e}")
        return results

    # ── New ───────────────────────────────────────────────────────

    def new_file(self, name: str = "Untitled") -> PrintFileData:
        """
        Create a new blank print file and set as current.

        Auto-increments name if a file with the same name exists.
        """
        # Ensure unique name
        base_name = name
        counter = 1
        existing = {info["name"] for info in self.list_files()}
        while name in existing:
            counter += 1
            name = f"{base_name}_{counter}"

        self.current = PrintFileData(
            metadata=PrintFileMetadata(name=name),
        )
        filename = _sanitize_filename(name) + ".json"
        self.current_path = self.prints_dir / filename

        # Save immediately
        self.save()
        logger.info(f"Created new print file: {name} → {self.current_path}")
        return self.current

    # ── Save ──────────────────────────────────────────────────────

    def save(self) -> bool:
        """Save the current file to disk."""
        if self.current is None or self.current_path is None:
            logger.warning("No file loaded to save")
            return False

        try:
            data = self.current.to_dict()
            with open(self.current_path, "w") as f:
                json.dump(data, f, indent=2)
            self.current.mark_clean()
            self._last_save_time = time.time()
            logger.debug(f"Saved: {self.current_path}")
            return True
        except OSError as e:
            logger.error(f"Failed to save {self.current_path}: {e}")
            return False

    def save_as(self, new_name: str) -> bool:
        """Save current file under a new name."""
        if self.current is None:
            return False

        self.current.metadata.name = new_name
        filename = _sanitize_filename(new_name) + ".json"
        self.current_path = self.prints_dir / filename
        return self.save()

    # ── Load ──────────────────────────────────────────────────────

    def load(self, name: str) -> PrintFileData | None:
        """
        Load a print file by name.

        Searches for a matching .json file, validates and migrates schema.
        """
        # Find file by name
        target_path = None
        for fp in self.prints_dir.glob("*.json"):
            try:
                with open(fp) as f:
                    data = json.load(f)
                if data.get("metadata", {}).get("name") == name:
                    target_path = fp
                    break
            except (json.JSONDecodeError, OSError):
                continue

        # Fallback: try sanitized filename
        if target_path is None:
            candidate = self.prints_dir / (_sanitize_filename(name) + ".json")
            if candidate.exists():
                target_path = candidate

        if target_path is None:
            logger.warning(f"Print file not found: {name}")
            return None

        try:
            with open(target_path) as f:
                raw = json.load(f)

            pre_migration_version = raw.get("schema_version", "7.1")
            pre_migration_snapshot = (
                copy.deepcopy(raw) if pre_migration_version < SCHEMA_VERSION
                else None
            )

            # Validate
            valid, errors = validate_print_file(raw)
            if not valid:
                # Try migration first
                raw = migrate_print_file(raw)
                valid, errors = validate_print_file(raw)
                if not valid:
                    logger.error(f"Invalid print file after migration: {errors}")
                    return None

            # Migrate if needed
            if pre_migration_version != SCHEMA_VERSION:
                raw = migrate_print_file(raw)
                # Persist a one-time pre-migration backup so a user can
                # roll back if anything regenerated unexpectedly.
                if pre_migration_snapshot is not None:
                    write_migration_backup(target_path, pre_migration_snapshot)

            self.current = PrintFileData.from_dict(raw)
            self.current_path = target_path
            logger.info(f"Loaded: {name} from {target_path}")
            return self.current

        except (json.JSONDecodeError, OSError) as e:
            logger.error(f"Failed to load {target_path}: {e}")
            return None

    # ── Delete ────────────────────────────────────────────────────

    def delete(self, name: str) -> bool:
        """Delete a print file by name."""
        for fp in self.prints_dir.glob("*.json"):
            try:
                with open(fp) as f:
                    data = json.load(f)
                if data.get("metadata", {}).get("name") == name:
                    fp.unlink()
                    if self.current and self.current.name == name:
                        self.current = None
                        self.current_path = None
                    logger.info(f"Deleted print file: {name}")
                    return True
            except (json.JSONDecodeError, OSError):
                continue

        # Fallback
        candidate = self.prints_dir / (_sanitize_filename(name) + ".json")
        if candidate.exists():
            candidate.unlink()
            if self.current and self.current.name == name:
                self.current = None
                self.current_path = None
            logger.info(f"Deleted print file: {name}")
            return True

        logger.warning(f"File not found for deletion: {name}")
        return False

    # ── Duplicate ─────────────────────────────────────────────────

    def duplicate(self, new_name: str) -> PrintFileData | None:
        """Deep copy current file under a new name."""
        if self.current is None:
            return None

        dup_data = copy.deepcopy(self.current.to_dict())
        dup_data["metadata"]["name"] = new_name
        now = datetime.now(timezone.utc).isoformat()
        dup_data["metadata"]["created"] = now
        dup_data["metadata"]["modified"] = now

        self.current = PrintFileData.from_dict(dup_data)
        filename = _sanitize_filename(new_name) + ".json"
        self.current_path = self.prints_dir / filename
        self.save()
        logger.info(f"Duplicated as: {new_name}")
        return self.current

    # ── Object CRUD ───────────────────────────────────────────────

    def add_object(self, name: str, obj_data: dict) -> str:
        """
        Add an object to the current file.

        Auto-increments name if duplicate. Returns the final name used.
        """
        if self.current is None:
            raise RuntimeError("No file loaded")

        # Ensure unique name
        base = name
        counter = 1
        while name in self.current.objects:
            counter += 1
            name = f"{base}_{counter}"

        self.current.objects[name] = obj_data
        self.current.mark_dirty()
        return name

    def remove_object(self, name: str) -> bool:
        """Remove an object by name."""
        if self.current is None:
            return False
        if name in self.current.objects:
            del self.current.objects[name]
            # Also remove from collections
            for coll_items in self.current.collections.values():
                coll_items[:] = [
                    item for item in coll_items
                    if item.get("object_name") != name
                ]
            self.current.mark_dirty()
            return True
        return False

    def update_object(self, name: str, obj_data: dict) -> bool:
        """Update an existing object's data."""
        if self.current is None or name not in self.current.objects:
            return False
        self.current.objects[name] = obj_data
        self.current.mark_dirty()
        return True

    # ── Collection CRUD ───────────────────────────────────────────

    def add_collection(self, name: str, items: list[dict] | None = None) -> str:
        """Add a named collection. Returns final name."""
        if self.current is None:
            raise RuntimeError("No file loaded")

        base = name
        counter = 1
        while name in self.current.collections:
            counter += 1
            name = f"{base}_{counter}"

        self.current.collections[name] = items or []
        self.current.mark_dirty()
        return name

    def remove_collection(self, name: str) -> bool:
        """Remove a collection by name."""
        if self.current is None:
            return False
        if name in self.current.collections:
            del self.current.collections[name]
            self.current.mark_dirty()
            return True
        return False

    # ── Layout Presets ────────────────────────────────────────────

    def add_layout_preset(self, name: str, pattern: str, params: dict) -> str:
        """Save a layout preset."""
        if self.current is None:
            raise RuntimeError("No file loaded")

        base = name
        counter = 1
        while name in self.current.layout_presets:
            counter += 1
            name = f"{base}_{counter}"

        self.current.layout_presets[name] = {
            "pattern": pattern,
            "params": params,
        }
        self.current.mark_dirty()
        return name

    def remove_layout_preset(self, name: str) -> bool:
        """Remove a layout preset."""
        if self.current is None:
            return False
        if name in self.current.layout_presets:
            del self.current.layout_presets[name]
            self.current.mark_dirty()
            return True
        return False

    # ── Auto-Save ─────────────────────────────────────────────────

    def auto_save(self, interval_s: float = 30.0) -> bool:
        """
        Save if dirty and enough time has passed since last save.

        Returns True if a save was performed.
        """
        if self.current is None or not self.current.is_dirty:
            return False
        if (time.time() - self._last_save_time) < interval_s:
            return False
        return self.save()

    # ── Utility ───────────────────────────────────────────────────

    def get_object_count(self) -> int:
        """Number of objects in current file."""
        return len(self.current.objects) if self.current else 0

    def get_collection_names(self) -> list[str]:
        """List collection names in current file."""
        return list(self.current.collections.keys()) if self.current else []

    def get_object_names(self) -> list[str]:
        """List object names in current file."""
        return list(self.current.objects.keys()) if self.current else []


# ═══════════════════════════════════════════════════════════════════
# Trajectory → custom print object  (v7.5.x — Print Builder)
# ═══════════════════════════════════════════════════════════════════

def save_trajectory_as_print_object(
    trajectory,
    base_name: str = "PrintBuilder",
    description: str = "",
    color: str = "#89b4fa",
    author: str = "Print Builder",
    source: str = "PrintBuilder",
    object_name: str = "Sketch_1",
    prints_dir: str = DEFAULT_PRINTS_DIR,
    extra_params: dict | None = None,
) -> str:
    """Bake an Nx7 trajectory into a ``csv_import`` print file.

    Writes ``{prints_dir}/{name}.csv`` (header ``x,y,z,p1,p2,p3,t``) and a
    matching ``{name}.json`` print file whose single object has
    ``object_type="csv_import"`` pointing at the CSV. The name is auto-
    incremented from ``base_name`` so existing files are never clobbered.

    This is the shared contract used by both the Print Builder Sketch page
    and the Image Import (Helper Functions) page so the resulting object
    shows up in Print Setup's custom-prints area. Returns the file name
    (no extension) for emitting via ``print_file_created``.
    """
    import csv as _csv
    import numpy as _np

    arr = _np.asarray(trajectory, dtype=_np.float64)
    if arr.ndim != 2 or arr.shape[1] < 7:
        raise ValueError(
            f"trajectory must be an Nx7 array, got shape {arr.shape}")
    arr = arr[:, :7]
    if len(arr) < 2:
        raise ValueError("trajectory must have at least 2 waypoints")

    out_dir = Path(prints_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    base = _sanitize_filename(base_name)
    counter = 1
    while (out_dir / f"{base}_{counter}.csv").exists() \
            or (out_dir / f"{base}_{counter}.json").exists():
        counter += 1
    name = f"{base}_{counter}"
    csv_path = out_dir / f"{name}.csv"
    json_path = out_dir / f"{name}.json"

    # ── CSV ──
    with open(csv_path, "w", newline="") as f:
        writer = _csv.writer(f)
        writer.writerow(["x", "y", "z", "p1", "p2", "p3", "t"])
        for row in arr:
            writer.writerow([f"{v:.6f}" for v in row])

    # ── Print file JSON (csv_import object) ──
    # Key the CSV pointer as ``source_file`` — the contract every csv_import
    # reader understands (matches the manual-import path in print_objects.py).
    # ``csv_path`` is kept as an alias for back-compat / human readability.
    params = {"source_file": str(csv_path), "csv_path": str(csv_path),
              "source": source, "num_waypoints": int(len(arr))}
    if extra_params:
        params.update(extra_params)

    now = datetime.now(timezone.utc).isoformat()
    print_data = {
        "schema_version": SCHEMA_VERSION,
        "metadata": {
            "name": name,
            "description": description or f"{author}: {name}",
            "created": now,
            "modified": now,
            "author": author,
        },
        "objects": {
            object_name: {
                "object_type": "csv_import",
                "params": params,
                "position": [0.0, 0.0, 0.0],
                "color": color,
                "ink": "",
                "in_well": True,
            },
        },
        "collections": {},
        "layout_presets": {},
    }
    with open(json_path, "w") as f:
        json.dump(print_data, f, indent=2)

    logger.info(f"save_trajectory_as_print_object → {name} "
                f"({len(arr)} waypoints)")
    return name
