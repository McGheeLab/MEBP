"""
validation.py — ValidationIssue dataclass for the Print Setup wizard.

A ``ValidationIssue`` is the unit consumed by the slide-up Validation
Panel. Each step in the wizard exposes a ``validate()`` method that
returns ``list[ValidationIssue]``; the orchestrator aggregates and
renders them.

Each issue carries enough context for the "Fix →" button to jump:
``step`` is the wizard step index, ``target_id`` is a string the step
can resolve to a focusable sub-widget via ``focus_target()``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ValidationSeverity(str, Enum):
    ERROR = "error"
    WARN = "warn"
    INFO = "info"


@dataclass
class ValidationIssue:
    severity: ValidationSeverity
    step: int
    target_id: str
    title: str
    detail: str = ""

    @property
    def is_error(self) -> bool:
        return self.severity == ValidationSeverity.ERROR

    @property
    def is_warn(self) -> bool:
        return self.severity == ValidationSeverity.WARN

    def to_dict(self) -> dict:
        return {
            "severity": self.severity.value,
            "step": self.step,
            "target_id": self.target_id,
            "title": self.title,
            "detail": self.detail,
        }


def aggregate(issues_per_step: list[list[ValidationIssue]]) -> list[ValidationIssue]:
    """Flatten per-step issue lists into a single aggregate list,
    preserving step order. Errors sort before warns sort before infos
    within each step."""
    rank = {
        ValidationSeverity.ERROR: 0,
        ValidationSeverity.WARN: 1,
        ValidationSeverity.INFO: 2,
    }
    result: list[ValidationIssue] = []
    for step_issues in issues_per_step:
        result.extend(sorted(step_issues, key=lambda i: rank.get(i.severity, 99)))
    return result
