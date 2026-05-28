"""
gui.pages.print_setup — Wizard-style Print Setup page (v7.5.0).

This package replaces the flat ``gui/pages/print_setup.py`` (4-tab) +
``gui/pages/print_workspace.py`` modules with a 4-step linear wizard
plus a persistent Print Objects side panel and on-demand validation
panel.

The page-level export is ``PrintSetupPage``; importers continue to
write ``from gui.pages.print_setup import PrintSetupPage``.

Submodules:
    wizard_step_base — WizardStepBase abstract widget
    models           — shared PrintObjectsModel, WellAssignmentModel
    validation       — ValidationIssue dataclass
    stepper          — top WizardStepper breadcrumb (TBD)
    page             — PrintSetupPage orchestrator (TBD)
    step_workspace   — Step 1 (TBD)
    step_objects     — Step 2 (TBD)
    step_wells       — Step 3 (TBD)
    step_plan        — Step 4 (TBD)
    side_panel_objects — persistent right rail (TBD)
    validation_panel — slide-up issue list (TBD)
    session          — PrintSession bridging (TBD; talks to
                       SupportClasses.PrintSessionManager)
"""

from .validation import ValidationIssue, ValidationSeverity
from .wizard_step_base import WizardStepBase

# ── Default export ──────────────────────────────────────────────────
# v7.5.0: the wizard orchestrator is now the default. It composes the
# legacy ``PrintSetupPage`` (now in ``print_setup_legacy.py``)
# internally and reparents its tab widgets into wizard steps, so
# behavior is identical to v7.4.2 while the wizard shell / side panel
# / validation panel are added on top. The legacy class remains
# importable as ``LegacyPrintSetupPage`` if you need to fall back
# during debugging.

from gui.pages.print_setup_legacy import (
    PrintSetupPage as LegacyPrintSetupPage,
    PrintSignalBridge,
)
from .page import WizardPrintSetupPage   # noqa: E402

PrintSetupPage = WizardPrintSetupPage

__all__ = [
    "ValidationIssue",
    "ValidationSeverity",
    "WizardStepBase",
    "PrintSetupPage",
    "LegacyPrintSetupPage",
    "WizardPrintSetupPage",
    "PrintSignalBridge",
]
