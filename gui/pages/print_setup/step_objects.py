"""
step_objects.py — Step 2 of the Print Setup wizard.

Wraps the existing ``PrintObjectsTab`` (``gui/pages/print_objects.py``)
as a wizard step. Mirrors the inner widget's collection-changed signal
through ``state_changed`` so the orchestrator can refresh the side
panel browser and Step 3's print-collection dropdowns.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QVBoxLayout

from .validation import ValidationIssue, ValidationSeverity
from .wizard_step_base import WizardStepBase


class ObjectsStep(WizardStepBase):
    """Step 2: design / import print objects + organize into
    collections."""

    step_index = 2
    step_title = "Print Objects"

    prints_changed = Signal(list)
    collections_changed = Signal(list)

    def __init__(self, controller: Any, settings: Any = None,
                 parent=None) -> None:
        super().__init__(parent)
        self._hardware_config = None

        from gui.pages.print_objects import PrintObjectsTab
        self._inner = PrintObjectsTab(
            controller=controller,
            settings=settings,
            parent=self,
        )
        # Mirror the inner widget's signals out + drive state_changed
        if hasattr(self._inner, "prints_changed"):
            self._inner.prints_changed.connect(self.prints_changed.emit)
            self._inner.prints_changed.connect(
                lambda _: self.state_changed.emit())
        if hasattr(self._inner, "collections_changed"):
            self._inner.collections_changed.connect(self.collections_changed.emit)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._inner)

    # ── Pass-through ──────────────────────────────────────────────

    def on_hardware_config_changed(self, hw_config) -> None:
        self._hardware_config = hw_config
        if hasattr(self._inner, "set_hardware_config"):
            self._inner.set_hardware_config(hw_config)

    def set_workspace(self, workspace) -> None:
        if hasattr(self._inner, "set_workspace"):
            self._inner.set_workspace(workspace)

    @property
    def inner(self):
        return self._inner

    def collection_names(self) -> list[str]:
        if hasattr(self._inner, "_collections"):
            return list(getattr(self._inner, "_collections", {}).keys())
        return []

    def object_names(self) -> list[str]:
        return [o.get("name", "")
                for o in getattr(self._inner, "_objects", [])]

    # ── WizardStepBase ────────────────────────────────────────────

    def is_valid(self) -> bool:
        return len(self.object_names()) > 0

    def validate(self) -> list[ValidationIssue]:
        issues: list[ValidationIssue] = []
        if not self.object_names():
            issues.append(ValidationIssue(
                severity=ValidationSeverity.ERROR,
                step=self.step_index,
                target_id="objects.empty",
                title="No print objects defined",
                detail="Add at least one object before continuing.",
            ))
            return issues
        # Per-object sanity
        for obj in getattr(self._inner, "_objects", []):
            if not obj.get("object_type"):
                issues.append(ValidationIssue(
                    severity=ValidationSeverity.ERROR,
                    step=self.step_index,
                    target_id=f"object:{obj.get('name', '?')}",
                    title=f"Object '{obj.get('name', '?')}' has no type",
                    detail="Pick a parametric shape or import a CSV.",
                ))
            if not obj.get("ink") and not obj.get("ink_pump"):
                issues.append(ValidationIssue(
                    severity=ValidationSeverity.WARN,
                    step=self.step_index,
                    target_id=f"object:{obj.get('name', '?')}",
                    title=f"Object '{obj.get('name', '?')}' has no ink",
                    detail="Assign an ink so the planner can compute flow.",
                ))
        return issues

    def get_state(self) -> dict:
        return {
            "objects": list(getattr(self._inner, "_objects", [])),
            "collections": dict(getattr(self._inner, "_collections", {})),
        }

    def set_state(self, state: dict) -> None:
        if not state:
            return
        if hasattr(self._inner, "_objects"):
            self._inner._objects = list(state.get("objects", []))
        if hasattr(self._inner, "_collections"):
            self._inner._collections = dict(state.get("collections", {}))
        # Best-effort refresh
        if hasattr(self._inner, "_refresh_object_list"):
            self._inner._refresh_object_list()
        if hasattr(self._inner, "_emit_prints_changed"):
            self._inner._emit_prints_changed()
