"""
step_workspace.py — Step 1 of the Print Setup wizard.

Thin WizardStepBase wrapper around the existing ``WorkspaceTab``
(``gui/pages/print_workspace.py``). The wrapper preserves all of the
existing widget's behavior (HW summary, plate format, "Edit Hardware
Setup" navigation) while exposing the wizard contract (``is_valid``,
``validate``, ``get_state``, ``set_state``).
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QVBoxLayout

from .validation import ValidationIssue, ValidationSeverity
from .wizard_step_base import WizardStepBase


class WorkspaceStep(WizardStepBase):
    """Step 1: confirm workspace + plate."""

    step_index = 1
    step_title = "Workspace"

    # Forwarded from the embedded WorkspaceTab so the orchestrator can
    # surface the same signals it does today.
    workspace_changed = Signal(object)  # WorkspaceConfig
    navigate_to_page = Signal(int)

    def __init__(self, controller: Any, settings: Any = None,
                 parent=None) -> None:
        super().__init__(parent)
        self._hardware_config = None

        # Lazy import to avoid pulling print_workspace.py at module
        # import time (it has heavy Qt deps).
        from gui.pages.print_workspace import WorkspaceTab

        self._inner = WorkspaceTab(
            controller=controller,
            settings=settings,
            parent=self,
        )
        # Forward signals
        if hasattr(self._inner, "workspace_changed"):
            self._inner.workspace_changed.connect(self.workspace_changed.emit)
            self._inner.workspace_changed.connect(
                lambda _ws: self.state_changed.emit()
            )
        if hasattr(self._inner, "navigate_to_page"):
            self._inner.navigate_to_page.connect(self.navigate_to_page.emit)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._inner)

    # ── Pass-through API ──────────────────────────────────────────

    def on_hardware_config_changed(self, hw_config) -> None:
        self._hardware_config = hw_config
        if hasattr(self._inner, "set_hardware_config"):
            self._inner.set_hardware_config(hw_config)

    @property
    def inner(self):  # exposed for orchestrator wiring
        return self._inner

    @property
    def workspace(self):
        return getattr(self._inner, "workspace", None)

    # ── WizardStepBase contract ───────────────────────────────────

    def is_valid(self) -> bool:
        return self._hardware_config is not None

    def validate(self) -> list[ValidationIssue]:
        if self._hardware_config is None:
            return [ValidationIssue(
                severity=ValidationSeverity.ERROR,
                step=self.step_index,
                target_id="workspace.hw_config",
                title="No hardware configuration",
                detail="Configure hardware on the Hardware Setup page before "
                       "starting a print session.",
            )]
        return []

    def get_state(self) -> dict:
        ws = self.workspace
        if ws is not None and hasattr(ws, "to_dict"):
            return ws.to_dict()
        return {}

    def set_state(self, state: dict) -> None:
        if not state or not hasattr(self._inner, "workspace"):
            return
        try:
            from SupportClasses.PhysicalModels import WorkspaceConfig
            if hasattr(WorkspaceConfig, "from_dict"):
                ws = WorkspaceConfig.from_dict(state)
                self._inner.workspace = ws
                if hasattr(self._inner, "_refresh_from_workspace"):
                    self._inner._refresh_from_workspace()
        except Exception:
            pass
