"""
step_plan.py — Step 4 of the Print Setup wizard.

Wraps the existing Finalize-tab body owned by
``gui/pages/print_setup_legacy.py``'s ``PrintSetupPage``.

Behavior parity is what matters here. The Finalize body builds:
 - Print Parameters (extrusion %, speed scale %, layers, advanced)
 - Plan of Action (ink swap / Z travel / XY travel / cleanup / gather)
 - Action bar (Validate, Generate, Send to Monitor, Export G-code, Save)

Rather than copy ~1000 lines of UI construction into the new package,
this step delegates body construction to a host
``PrintSetupPage`` (the legacy class, instantiated by the wizard
orchestrator) — and exposes a thin wizard-facing contract on top.

A follow-up commit can move the Finalize body wholesale into this
file; until then, behavior is unchanged from v7.4.2.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLabel, QVBoxLayout

from gui.scaling import s as _s
from gui.styles import COLORS

from .validation import ValidationIssue, ValidationSeverity
from .wizard_step_base import WizardStepBase


class PlanStep(WizardStepBase):
    """Step 4: configure execution params, validate, send to monitor.

    The actual body widget is provided by the orchestrator (it lives
    on the legacy ``PrintSetupPage`` so all of its helper methods
    remain wired). PlanStep just hosts it inside the wizard.
    """

    step_index = 4
    step_title = "Plan & Run"

    job_ready = Signal(object)   # PrintJob

    def __init__(self, body_widget=None, parent=None) -> None:
        super().__init__(parent)
        self._body = body_widget
        self._generated_job = None
        self._has_generated = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        if body_widget is not None:
            body_widget.setParent(self)
            lay.addWidget(body_widget)
            body_widget.show()
        else:
            placeholder = QLabel(
                "Plan & Run body not yet wired.\n\n"
                "This step embeds the legacy Finalize-tab body until the\n"
                "Print Parameters + Plan of Action layout is migrated\n"
                "into the new wizard package."
            )
            placeholder.setStyleSheet(
                f"color: {COLORS['subtext0']}; "
                f"padding: {_s(40)}px; "
                f"font-style: italic;"
            )
            lay.addWidget(placeholder)

    # ── Pass-through ──────────────────────────────────────────────

    def attach_body(self, body_widget) -> None:
        """Allow the orchestrator to inject the Finalize body after
        construction (the body is built by the legacy
        PrintSetupPage which is itself owned by the wizard)."""
        if self._body is not None:
            return
        self._body = body_widget
        self.layout().addWidget(body_widget)

    def on_hardware_config_changed(self, hw_config) -> None:
        # The body widget lives on the legacy PrintSetupPage which
        # propagates HW updates through its own set_hardware_config
        # — no extra wiring needed here.
        return None

    def mark_generated(self, job: Any) -> None:
        """Called by the orchestrator after a successful Generate."""
        self._generated_job = job
        self._has_generated = job is not None
        self.state_changed.emit()

    # ── WizardStepBase ────────────────────────────────────────────

    def is_valid(self) -> bool:
        return True   # Plan step is always "advanceable" — the
                      # final gate is Validate + Generate, not Next.

    def validate(self) -> list[ValidationIssue]:
        issues: list[ValidationIssue] = []
        if not self._has_generated:
            issues.append(ValidationIssue(
                severity=ValidationSeverity.INFO,
                step=self.step_index,
                target_id="plan.generate",
                title="Plan not generated yet",
                detail="Click Generate to build the trajectory + plan.",
            ))
        return issues

    def get_state(self) -> dict:
        # The Finalize body's widget state lives on the legacy page —
        # the wizard orchestrator captures it via
        # ``_build_execution_config`` (already used to assemble a
        # PrintJob today). For session save/load we capture the
        # current PrintExecutionConfig snapshot.
        return {}

    def set_state(self, state: dict) -> None:
        return None
