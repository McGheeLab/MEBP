"""
step_wells.py — Step 3 of the Print Setup wizard.

Wraps the existing ``WellSetupTab`` so the user can assign roles to
wells and bind print collections. The wrapper preserves all existing
behavior (selection, role bar, save/load) and exposes the wizard
contract (``is_valid`` / ``validate`` / ``get_state`` / ``set_state``).

The "single-click + inspector" UX improvement called out in the v7.5.0
plan is left for a follow-up commit; this step ships with the existing
4-click role flow so the wizard works end-to-end without regressing.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QVBoxLayout

from .validation import ValidationIssue, ValidationSeverity
from .wizard_step_base import WizardStepBase


class WellsStep(WizardStepBase):
    """Step 3: assign wells to roles + bind print collections."""

    step_index = 3
    step_title = "Wells & Roles"

    setup_changed = Signal()

    def __init__(self, controller: Any, settings: Any = None,
                 workspace=None, parent=None) -> None:
        super().__init__(parent)
        self._hardware_config = None

        from gui.pages.print_well_setup import WellSetupTab
        self._inner = WellSetupTab(
            controller=controller,
            settings=settings,
            workspace=workspace,
            parent=self,
        )
        if hasattr(self._inner, "setup_changed"):
            self._inner.setup_changed.connect(self.setup_changed.emit)
            self._inner.setup_changed.connect(
                lambda: self.state_changed.emit())

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

    def set_available_prints(self, names: list[str]) -> None:
        if hasattr(self._inner, "set_available_prints"):
            self._inner.set_available_prints(names)

    def update_needle_position(self, x_mm, y_mm, z_mm=None) -> None:
        if hasattr(self._inner, "update_needle_position"):
            self._inner.update_needle_position(x_mm, y_mm, z_mm)

    @property
    def inner(self):
        return self._inner

    @property
    def model(self):
        return getattr(self._inner, "model", None)

    # ── WizardStepBase ────────────────────────────────────────────

    def _assignments(self) -> dict[str, Any]:
        model = self.model
        if model is None:
            return {}
        return getattr(model, "assignments", {}) or {}

    def _wells_with_role(self, role: str) -> list[str]:
        out: list[str] = []
        for well, wa in self._assignments().items():
            wa_role = getattr(wa, "role", None)
            if wa_role is None:
                continue
            value = getattr(wa_role, "value", str(wa_role)).lower()
            if value == role.lower():
                out.append(well)
        return out

    def is_valid(self) -> bool:
        return bool(self._wells_with_role("print"))

    def validate(self) -> list[ValidationIssue]:
        issues: list[ValidationIssue] = []
        print_wells = self._wells_with_role("print")
        if not print_wells:
            issues.append(ValidationIssue(
                severity=ValidationSeverity.ERROR,
                step=self.step_index,
                target_id="wells.no_print",
                title="No print well assigned",
                detail="Pick at least one well and give it the Print role.",
            ))
            return issues
        # Each print well should reference at least one collection
        for well in print_wells:
            wa = self._assignments().get(well)
            refs = getattr(wa, "print_collections", []) or []
            if not refs:
                issues.append(ValidationIssue(
                    severity=ValidationSeverity.ERROR,
                    step=self.step_index,
                    target_id=f"well:{well}",
                    title=f"Print well {well} has no collection",
                    detail="Pick a print file/collection for this well.",
                ))
        # Soft check: waste well exists
        if not self._wells_with_role("waste"):
            issues.append(ValidationIssue(
                severity=ValidationSeverity.WARN,
                step=self.step_index,
                target_id="wells.no_waste",
                title="No waste well assigned",
                detail="Most ink-swap sequences need a waste well. "
                       "Assign one or disable waste in the plan.",
            ))
        return issues

    def get_state(self) -> dict:
        model = self.model
        if model is None or not hasattr(model, "to_dict"):
            return {}
        try:
            return model.to_dict()
        except Exception:
            return {}

    def set_state(self, state: dict) -> None:
        model = self.model
        if model is None or not state:
            return
        if hasattr(model, "from_dict"):
            try:
                # Some WellSetupModel.from_dict is a classmethod returning
                # a new model; if so we replace; otherwise we update in
                # place via a "load" helper.
                loaded = type(model).from_dict(state)
                if hasattr(self._inner, "_model"):
                    self._inner._model = loaded
                if hasattr(self._inner, "_refresh_plate"):
                    self._inner._refresh_plate()
            except Exception:
                pass

    def focus_target(self, target_id: str) -> None:
        if not target_id.startswith("well:"):
            return
        well = target_id.split(":", 1)[1]
        pv = getattr(self._inner, "plate_view", None)
        if pv is not None and hasattr(pv, "select_well"):
            try:
                pv.select_well(well)
            except Exception:
                pass
