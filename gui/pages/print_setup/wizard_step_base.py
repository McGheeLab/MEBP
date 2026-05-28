"""
wizard_step_base.py — WizardStepBase abstract widget.

Contract every Print Setup wizard step satisfies. The orchestrator
relies on this contract for advancing/validating/serializing state,
without caring about each step's internal layout.

Steps own their widgets but NOT their underlying data — shared models
(PrintObjectsModel, WellAssignmentModel) live at the orchestrator and
are passed in. This keeps cross-step state consistent: when an object
is deleted in Step 2's body, the Step 3 well inspector and the
persistent side panel both react via Qt model signals.
"""

from __future__ import annotations

from typing import Iterable

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget

from .validation import ValidationIssue


class WizardStepBase(QWidget):
    """Abstract base for wizard step bodies.

    Subclasses MUST override:
        is_valid()           — quick "can advance" check
        validate()           — deep validation returning ValidationIssues
        get_state()          — serializable dict of step state
        set_state(d)         — restore from a dict produced by get_state
        focus_target(tid)    — best-effort focus on a sub-widget

    Subclasses SHOULD emit ``state_changed`` whenever their state
    mutates so the orchestrator can re-evaluate progress and dirty-mark
    the session.
    """

    #: Emitted whenever this step's serializable state changes.
    state_changed = Signal()

    #: Subclasses set this to the wizard step index (1-based).
    step_index: int = 0

    #: Human-readable step title; shown in the WizardStep header.
    step_title: str = ""

    # ── Required overrides ────────────────────────────────────────

    def is_valid(self) -> bool:
        """Lightweight predicate for whether the user may advance."""
        raise NotImplementedError

    def validate(self) -> list[ValidationIssue]:
        """Return a (possibly empty) list of issues."""
        raise NotImplementedError

    def get_state(self) -> dict:
        """Return a JSON-serializable snapshot of this step's state."""
        raise NotImplementedError

    def set_state(self, state: dict) -> None:
        """Restore from a dict produced by ``get_state``. Must not
        emit ``state_changed`` during restoration."""
        raise NotImplementedError

    def focus_target(self, target_id: str) -> None:  # pragma: no cover - default
        """Default no-op. Steps that emit issues with target_ids should
        override to scroll/flash the relevant sub-widget."""
        return None

    # ── Optional lifecycle hooks ──────────────────────────────────

    def on_enter(self) -> None:  # pragma: no cover - default
        """Called by the orchestrator when this step becomes active."""
        return None

    def on_leave(self) -> None:  # pragma: no cover - default
        """Called when leaving this step (either Prev or Next)."""
        return None

    def on_hardware_config_changed(self, hw_config) -> None:  # pragma: no cover
        """Called when the active HardwareConfig changes (e.g. after
        the user accepts an InvalidationBanner refresh). Steps that
        derive defaults from HW config should override."""
        return None

    # ── Helpers for issue construction ────────────────────────────

    def _issues(self, *issues: ValidationIssue) -> list[ValidationIssue]:
        """Convenience for ``validate()`` implementations that want to
        early-exit with a single list."""
        return list(issues)

    @staticmethod
    def collect_issues(*groups: Iterable[ValidationIssue]) -> list[ValidationIssue]:
        """Flatten multiple iterables of issues into a single list."""
        out: list[ValidationIssue] = []
        for g in groups:
            out.extend(g)
        return out
