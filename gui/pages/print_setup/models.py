"""
models.py — Shared Qt models for the Print Setup wizard.

The wizard's cross-step coherence depends on shared state: when an
object is added in Step 2's body, the side panel browser must update
immediately, and Step 3's well inspector must see the new object as a
valid bind target. Rather than have each widget hold its own copy of
the list, we use these small Qt models as the single source of truth.

* ``PrintObjectsModel``  — list of parametric / csv print objects,
                            persisted into the session's ``objects``
                            and ``collections``.
* ``WellAssignmentModel`` — per-well role + ink + print-collection
                            references, mirrors ``WellSetupModel``
                            (SupportClasses.WellSetup) so the wizard
                            doesn't need to subclass the backend.

The models are intentionally thin: they store dicts (the
``PrintObject.to_dict()`` / ``WellAssignment.to_dict()`` shape) and
let widgets create real dataclass instances when needed via the
``from_dict`` constructors. This keeps the wizard portable across
backend revisions during Phase E (data-model hard cuts).
"""

from __future__ import annotations

import copy
import logging
from typing import Any

from PySide6.QtCore import QObject, Signal

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# PrintObjectsModel
# ═══════════════════════════════════════════════════════════════════


class PrintObjectsModel(QObject):
    """Shared store of print objects + collections for the wizard.

    Stores objects as raw dicts (one per object) plus a list of
    collection dicts. Emits ``changed`` whenever the store mutates.
    """

    changed = Signal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._objects: list[dict] = []          # PrintObject.to_dict() shape
        self._collections: list[dict] = []      # PrintCollection.to_dict() shape

    # ── Access ────────────────────────────────────────────────────

    def objects(self) -> list[dict]:
        return list(self._objects)

    def collections(self) -> list[dict]:
        return list(self._collections)

    def object_names(self) -> list[str]:
        return [o.get("name", "") for o in self._objects]

    def collection_names(self) -> list[str]:
        return [c.get("name", "") for c in self._collections]

    def find_object(self, name: str) -> dict | None:
        for o in self._objects:
            if o.get("name") == name:
                return o
        return None

    def find_collection(self, name: str) -> dict | None:
        for c in self._collections:
            if c.get("name") == name:
                return c
        return None

    # ── Mutators ──────────────────────────────────────────────────

    def add_object(self, obj_dict: dict) -> str:
        """Add an object. Auto-numbers the name if a duplicate exists.
        Returns the final name."""
        name = obj_dict.get("name", "Object")
        base = name
        existing = set(self.object_names())
        n = 1
        while name in existing:
            n += 1
            name = f"{base}_{n}"
        obj_dict = copy.deepcopy(obj_dict)
        obj_dict["name"] = name
        self._objects.append(obj_dict)
        self.changed.emit()
        return name

    def update_object(self, name: str, obj_dict: dict) -> bool:
        for i, o in enumerate(self._objects):
            if o.get("name") == name:
                new = copy.deepcopy(obj_dict)
                new["name"] = name
                self._objects[i] = new
                self.changed.emit()
                return True
        return False

    def remove_object(self, name: str) -> bool:
        before = len(self._objects)
        self._objects = [o for o in self._objects if o.get("name") != name]
        # Also drop from any collections that reference this object
        for c in self._collections:
            items = c.get("objects", [])
            c["objects"] = [
                it for it in items
                if (it.get("name") if isinstance(it, dict) else it) != name
            ]
        if len(self._objects) != before:
            self.changed.emit()
            return True
        return False

    def add_collection(self, coll_dict: dict) -> str:
        name = coll_dict.get("name", "Collection")
        base = name
        existing = set(self.collection_names())
        n = 1
        while name in existing:
            n += 1
            name = f"{base}_{n}"
        coll_dict = copy.deepcopy(coll_dict)
        coll_dict["name"] = name
        self._collections.append(coll_dict)
        self.changed.emit()
        return name

    def remove_collection(self, name: str) -> bool:
        before = len(self._collections)
        self._collections = [c for c in self._collections if c.get("name") != name]
        if len(self._collections) != before:
            self.changed.emit()
            return True
        return False

    # ── Serialization ─────────────────────────────────────────────

    def to_state(self) -> dict:
        return {
            "objects": copy.deepcopy(self._objects),
            "collections": copy.deepcopy(self._collections),
        }

    def set_state(self, state: dict) -> None:
        self._objects = list(state.get("objects", []))
        self._collections = list(state.get("collections", []))
        self.changed.emit()

    def clear(self) -> None:
        if self._objects or self._collections:
            self._objects.clear()
            self._collections.clear()
            self.changed.emit()


# ═══════════════════════════════════════════════════════════════════
# WellAssignmentModel
# ═══════════════════════════════════════════════════════════════════


class WellAssignmentModel(QObject):
    """Shared store of per-well assignments for the wizard.

    Mirrors the shape of ``WellSetupModel.to_dict()`` (from
    SupportClasses.WellSetup) so it can round-trip through the session
    file without any UI-specific transformations.
    """

    changed = Signal()
    well_changed = Signal(str)   # well_name

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._assignments: dict[str, dict] = {}
        self._calibration: dict = {}

    def assignments(self) -> dict[str, dict]:
        return copy.deepcopy(self._assignments)

    def get(self, well_name: str) -> dict | None:
        return self._assignments.get(well_name)

    def set(self, well_name: str, assignment: dict) -> None:
        self._assignments[well_name] = copy.deepcopy(assignment)
        self.well_changed.emit(well_name)
        self.changed.emit()

    def remove(self, well_name: str) -> bool:
        if well_name in self._assignments:
            del self._assignments[well_name]
            self.well_changed.emit(well_name)
            self.changed.emit()
            return True
        return False

    def wells_with_role(self, role: str) -> list[str]:
        return [
            w for w, a in self._assignments.items()
            if isinstance(a, dict) and a.get("role") == role
        ]

    def to_state(self) -> dict:
        return {
            "assignments": copy.deepcopy(self._assignments),
            "calibration": copy.deepcopy(self._calibration),
        }

    def set_state(self, state: dict) -> None:
        self._assignments = dict(state.get("assignments", {}))
        self._calibration = dict(state.get("calibration", {}))
        self.changed.emit()

    def clear(self) -> None:
        if self._assignments or self._calibration:
            self._assignments.clear()
            self._calibration.clear()
            self.changed.emit()
