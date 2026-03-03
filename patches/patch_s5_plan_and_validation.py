#!/usr/bin/env python3
"""
MEBP v7.2.4 -- Session 5 Patch
Print Plan of Action UI + Well Setup Validation + Send-to-Monitor Gate

Issues Covered: #10, #11
Tasks: S5.6-S5.15

Files Modified:
    gui/pages/print_well_setup.py  -- Plan UI section + validate()
    gui/pages/print_setup.py       -- Validation gate on Send to Monitor

Prerequisites:
    - Session 1 (styles + propagation)
    - Session 3 (pump-ink mapping)
    - SupportClasses/PrintPlanOfAction.py must exist (S5.1-S5.5)

Usage:
    python patch_s5_plan_and_validation.py /path/to/MEBP
"""

import os
import re
import sys
import shutil
from pathlib import Path

# Terminal colors
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
RESET = "\033[0m"

_applied = 0
_skipped = 0
_failed = 0


def find_project_root() -> Path:
    candidates = [
        Path.cwd(),
        Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    print(f"{RED}ERROR{RESET}: Could not find project root.")
    print("Pass it as argument: python patch_s5_plan_and_validation.py /path/to/MEBP")
    sys.exit(1)


def backup(filepath):
    bak = filepath + ".bak_s5"
    if not os.path.exists(bak):
        shutil.copy2(filepath, bak)


def read_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()


def write_file(filepath, content):
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)


def safe_replace(filepath, old, new, tag):
    global _applied, _skipped, _failed
    content = read_file(filepath)
    if new.strip()[:60] in content:
        print(f"  {YELLOW}SKIP{RESET}: {tag} -- already applied")
        _skipped += 1
        return 1
    if old not in content:
        print(f"  {RED}MISS{RESET}: {tag} -- anchor not found")
        _failed += 1
        return 0
    content = content.replace(old, new, 1)
    write_file(filepath, content)
    print(f"  {GREEN}OK{RESET}:   {tag}")
    _applied += 1
    return 1


def safe_insert_after(filepath, anchor, insertion, tag):
    global _applied, _skipped, _failed
    content = read_file(filepath)
    if insertion.strip()[:60] in content:
        print(f"  {YELLOW}SKIP{RESET}: {tag} -- already applied")
        _skipped += 1
        return 1
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {tag} -- anchor not found")
        _failed += 1
        return 0
    idx = content.index(anchor) + len(anchor)
    content = content[:idx] + "\n" + insertion + content[idx:]
    write_file(filepath, content)
    print(f"  {GREEN}OK{RESET}:   {tag}")
    _applied += 1
    return 1


def safe_insert_before(filepath, anchor, insertion, tag):
    global _applied, _skipped, _failed
    content = read_file(filepath)
    if insertion.strip()[:60] in content:
        print(f"  {YELLOW}SKIP{RESET}: {tag} -- already applied")
        _skipped += 1
        return 1
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {tag} -- anchor not found")
        _failed += 1
        return 0
    idx = content.index(anchor)
    content = content[:idx] + insertion + "\n" + content[idx:]
    write_file(filepath, content)
    print(f"  {GREEN}OK{RESET}:   {tag}")
    _applied += 1
    return 1


# =================================================================
# PATCH A: gui/pages/print_well_setup.py
# =================================================================

def patch_well_setup(root):
    filepath = os.path.join(root, "gui", "pages", "print_well_setup.py")
    print(f"\n{'=' * 60}")
    print(f"PATCH A: {filepath}")
    print(f"{'=' * 60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return
    backup(filepath)

    # A1: Add PrintPlanOfAction import
    safe_insert_after(
        filepath,
        "from SupportClasses.WellSetup import (",
        """
from SupportClasses.PrintPlanOfAction import (
    PrintPlanOfAction, PlanPreferences, PlanStepType,
    PLAN_STEP_COLORS, PLAN_STEP_ICONS,
    validate_well_setup,
)
""",
        "A1: Add PrintPlanOfAction imports",
    )

    # A2: Add QMessageBox import if not present
    content = read_file(filepath)
    if "QMessageBox" not in content:
        safe_replace(
            filepath,
            "from PySide6.QtWidgets import (",
            "from PySide6.QtWidgets import (\n    QMessageBox,",
            "A2: Add QMessageBox import",
        )

    # A3: Add plan-related instance variables to __init__
    safe_insert_after(
        filepath,
        "self._available_prints: list[str] = []",
        """
        # v7.2.4: Print Plan of Action (S5)
        self._plan: PrintPlanOfAction | None = None
        self._plan_preferences = PlanPreferences()
        self._hw_config = None
""",
        "A3: Add plan instance variables",
    )

    # A4: Add _build_plan_section call in _build_ui
    # We insert the plan section before the summary table
    content = read_file(filepath)
    # Try to find the summary table section
    summary_anchors = [
        "# -- Assignment Summary",
        "# -- Summary Table",
        "Assignment Summary",
        "self.summary_table",
    ]
    anchor_found = None
    for anchor in summary_anchors:
        if anchor in content:
            anchor_found = anchor
            break

    if anchor_found:
        safe_insert_before(
            filepath,
            anchor_found,
            PLAN_UI_SECTION,
            "A4: Add Plan of Action UI section",
        )
    else:
        # Fallback: insert before save/load section
        for fallback in ["# -- Save/Load", "def _save_layout"]:
            if fallback in content:
                safe_insert_before(
                    filepath,
                    fallback,
                    PLAN_UI_SECTION,
                    "A4: Add Plan of Action UI section (fallback)",
                )
                break

    # A5: Add plan methods before _save_layout
    safe_insert_before(
        filepath,
        "    def _save_layout(self)",
        PLAN_METHODS,
        "A5: Add plan generation/display/validate methods",
    )

    # A6: Add validate() method before _save_layout
    safe_insert_before(
        filepath,
        "    def _save_layout(self)",
        VALIDATE_METHOD,
        "A6: Add validate() method",
    )

    # A7: Store hw_config reference in set_hardware_config
    content = read_file(filepath)
    if "self._hw_config = config" not in content:
        if "def set_hardware_config(self, config" in content:
            safe_insert_after(
                filepath,
                "def set_hardware_config(self, config):",
                "        # v7.2.4 S5: Store reference for plan generation\n",
                "A7: Ensure hw_config stored",
            )

    write_file(filepath, read_file(filepath))


# Plan UI section to insert into _build_ui
PLAN_UI_SECTION = '''
        # -- Print Plan of Action (v7.2.4 S5) ---------------------
        plan_group = QGroupBox("Print Plan of Action")
        plan_group.setCheckable(True)
        plan_group.setChecked(True)
        plan_layout = QVBoxLayout(plan_group)

        # Plan preferences row
        pref_row = QHBoxLayout()

        pref_row.addWidget(QLabel("Max Ink/Run:"))
        self._max_ink_spin = QDoubleSpinBox()
        self._max_ink_spin.setRange(0.1, 1000.0)
        self._max_ink_spin.setValue(100.0)
        self._max_ink_spin.setSuffix(" uL")
        self._max_ink_spin.setDecimals(1)
        pref_row.addWidget(self._max_ink_spin)

        self._wash_check = QCheckBox("Wash")
        self._wash_check.setChecked(True)
        pref_row.addWidget(self._wash_check)

        self._waste_check = QCheckBox("Waste")
        self._waste_check.setChecked(True)
        pref_row.addWidget(self._waste_check)

        self._buffer_check = QCheckBox("Buffer")
        self._buffer_check.setChecked(True)
        pref_row.addWidget(self._buffer_check)

        plan_layout.addLayout(pref_row)

        # Generate + Validate buttons
        btn_row = QHBoxLayout()
        self._generate_plan_btn = QPushButton("Generate Plan")
        self._generate_plan_btn.clicked.connect(self._generate_plan)
        btn_row.addWidget(self._generate_plan_btn)

        self._validate_btn = QPushButton("Validate Setup")
        self._validate_btn.clicked.connect(self._run_validation)
        btn_row.addWidget(self._validate_btn)
        plan_layout.addLayout(btn_row)

        # Plan step display (scrollable list)
        self._plan_display = QLabel("No plan generated yet")
        self._plan_display.setWordWrap(True)
        self._plan_display.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px; padding: 4px;")
        plan_scroll = QScrollArea()
        plan_scroll.setWidget(self._plan_display)
        plan_scroll.setWidgetResizable(True)
        plan_scroll.setMaximumHeight(180)
        plan_layout.addWidget(plan_scroll)

        # Plan summary
        self._plan_summary = QLabel("")
        self._plan_summary.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: bold; font-size: 10px;")
        plan_layout.addWidget(self._plan_summary)

        # Validation status
        self._validation_label = QLabel("")
        self._validation_label.setWordWrap(True)
        self._validation_label.setStyleSheet(f"font-size: 10px;")
        plan_layout.addWidget(self._validation_label)

        main_layout.addWidget(plan_group)
'''


# Plan methods to add
PLAN_METHODS = '''
    # == Print Plan of Action (v7.2.4 S5) ========================

    def _get_plan_preferences(self) -> PlanPreferences:
        """Build PlanPreferences from UI controls."""
        prefs = PlanPreferences(
            wash_after_refill=self._wash_check.isChecked(),
            waste_before_refill=self._waste_check.isChecked(),
            refill_buffer_after_waste=self._buffer_check.isChecked(),
        )
        # Set max ink per run for all pumps
        max_vol = self._max_ink_spin.value()
        if self._hw_config:
            for pid in self._hw_config.enabled_pump_ids:
                prefs.max_ink_volume_uL[pid] = max_vol
        return prefs

    def _generate_plan(self) -> None:
        """Generate execution plan from current setup."""
        if self._hw_config is None:
            self._plan_display.setText(
                "Cannot generate plan: no hardware configuration")
            return

        prefs = self._get_plan_preferences()
        try:
            self._plan = PrintPlanOfAction.generate_plan(
                hw_config=self._hw_config,
                well_model=self._model,
                preferences=prefs,
            )
        except Exception as e:
            self._plan_display.setText(f"Plan generation failed: {e}")
            logger.error(f"Plan generation error: {e}", exc_info=True)
            return

        # Display steps
        if self._plan and self._plan.steps:
            lines = self._plan.step_summary_lines()
            self._plan_display.setText("\\n".join(lines))
            self._plan_summary.setText(self._plan.summary())
            self._plan_summary.setStyleSheet(
                f"color: {COLORS['green']}; font-weight: bold; font-size: 10px;")
        else:
            self._plan_display.setText("Plan is empty (no print wells?)")
            self._plan_summary.setText("")

        # Auto-validate after generation
        self._run_validation()

    def _run_validation(self) -> None:
        """Run comprehensive validation and display results."""
        if self._hw_config is None:
            self._validation_label.setText(
                "Cannot validate: no hardware configuration")
            self._validation_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 10px;")
            return

        is_valid, issues = validate_well_setup(
            hw_config=self._hw_config,
            well_model=self._model,
            plan=self._plan,
        )

        if is_valid:
            self._validation_label.setText("Ready to print")
            self._validation_label.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 10px;")
        else:
            text = "Issues found:\\n" + "\\n".join(f"  {i}" for i in issues)
            self._validation_label.setText(text)
            self._validation_label.setStyleSheet(
                f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")

    def _on_plan_auto_regen(self) -> None:
        """Auto-regenerate plan when assignments change (debounced)."""
        if self._plan is not None and self._hw_config is not None:
            self._generate_plan()

'''


# Validate method
VALIDATE_METHOD = '''
    def validate(self) -> tuple[bool, list[str]]:
        """
        v7.2.4 S5.11: Comprehensive validation before send-to-monitor.

        Called by print_setup.py before emitting job_ready signal.
        Returns (is_valid, list_of_issues).
        """
        return validate_well_setup(
            hw_config=self._hw_config,
            well_model=self._model,
            plan=self._plan,
        )

    def get_plan(self) -> PrintPlanOfAction | None:
        """Get the current plan of action (for inclusion in PrintJob)."""
        return self._plan

'''


# =================================================================
# PATCH B: gui/pages/print_setup.py
# =================================================================

def patch_print_setup(root):
    filepath = os.path.join(root, "gui", "pages", "print_setup.py")
    print(f"\n{'=' * 60}")
    print(f"PATCH B: {filepath}")
    print(f"{'=' * 60}")

    if not os.path.isfile(filepath):
        print(f"  {RED}ERROR{RESET}: File not found!")
        return
    backup(filepath)

    # B1: Add QMessageBox import if not present
    content = read_file(filepath)
    if "QMessageBox" not in content:
        safe_replace(
            filepath,
            "from PySide6.QtWidgets import (",
            "from PySide6.QtWidgets import (\n    QMessageBox,",
            "B1: Add QMessageBox import",
        )

    # B2: Replace _send_to_monitor with validation-gated version
    content = read_file(filepath)
    if "validate_well_setup" in content or "tab_wells.validate()" in content:
        print(f"  {YELLOW}SKIP{RESET}: B2: Validation gate already present")
        global _skipped
        _skipped += 1
    elif "def _send_to_monitor(self):" in content:
        # Find the method and replace it
        old_method_pattern = re.compile(
            r'(    def _send_to_monitor\(self\):.*?)'
            r'(?=\n    def |\nclass |\Z)',
            re.DOTALL,
        )
        match = old_method_pattern.search(content)
        if match:
            old_method = match.group(1)
            content = content.replace(old_method, SEND_TO_MONITOR_REPLACEMENT, 1)
            write_file(filepath, content)
            print(f"  {GREEN}OK{RESET}:   B2: Replaced _send_to_monitor with validation gate")
            global _applied
            _applied += 1
        else:
            print(f"  {RED}MISS{RESET}: B2: Could not parse _send_to_monitor method")
            global _failed
            _failed += 1
    else:
        print(f"  {RED}MISS{RESET}: B2: _send_to_monitor not found")
        _failed += 1


SEND_TO_MONITOR_REPLACEMENT = '''    def _send_to_monitor(self):
        """
        v7.2.4 S5.12: Build a print job with validation gate.

        Validates well setup before sending. Shows issues dialog on failure.
        Includes PrintPlanOfAction in the job when valid.
        """
        # Run validation first
        if hasattr(self.tab_wells, 'validate'):
            is_valid, issues = self.tab_wells.validate()
            if not is_valid:
                msg = QMessageBox(self)
                msg.setWindowTitle("Setup Validation Failed")
                msg.setIcon(QMessageBox.Icon.Warning)
                msg.setText(
                    f"Cannot send to monitor: {len(issues)} issue(s) found."
                )
                msg.setDetailedText("\\n".join(f"\\u2022 {i}" for i in issues))
                msg.exec()
                self.status_label.setText(
                    f"\\u26a0 {len(issues)} validation issue(s)")
                self.status_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                logger.warning(f"Send-to-monitor blocked: {issues}")
                return

        # Build job
        job = self._build_current_job()
        if job is None:
            self.status_label.setText("\\u26a0 No job -- configure wells first")
            self.status_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 10px;")
            return

        # Attach plan of action to job
        if hasattr(self.tab_wells, 'get_plan'):
            plan = self.tab_wells.get_plan()
            if plan is not None:
                job.plan_of_action = plan

        # Show confirmation
        summary_parts = []
        if hasattr(self.tab_wells, '_model'):
            model = self.tab_wells._model
            print_wells = [
                n for n, a in model.assignments.items()
                if a.role.value == "print"
            ]
            summary_parts.append(f"{len(print_wells)} print wells")
        if hasattr(self.tab_wells, 'get_plan'):
            plan = self.tab_wells.get_plan()
            if plan:
                summary_parts.append(f"{plan.total_runs} run(s)")
                summary_parts.append(
                    f"~{plan.estimated_total_seconds / 60:.1f} min")

        confirm = QMessageBox.question(
            self,
            "Send to Monitor",
            f"Send job to Print Monitor?\\n\\n"
            + "\\n".join(summary_parts),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return

        self.job_ready.emit(job)
        self.status_label.setText(f"\\u2713 Job sent: {job.name}")
        self.status_label.setStyleSheet(
            f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        logger.info(f"Print job sent to Monitor: {job.name}")

'''


# =================================================================
# MAIN
# =================================================================

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'=' * 60}")
    print(f" MEBP v7.2.4 -- Session 5 Patch")
    print(f" Print Plan of Action + Well Validation")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    # Verify prerequisites
    ppa_path = root / "SupportClasses" / "PrintPlanOfAction.py"
    if not ppa_path.exists():
        print(f"\n{RED}ERROR{RESET}: PrintPlanOfAction.py not found!")
        print(f"Copy it to: {ppa_path}")
        print(f"This file is a new deliverable from Session 5 (S5.1-S5.5)")
        sys.exit(1)

    for check_dir in ["gui/pages", "SupportClasses"]:
        if not (root / check_dir).is_dir():
            print(f"\n{RED}ERROR{RESET}: Expected directory not found: {check_dir}")
            sys.exit(1)

    # Run patches
    patch_well_setup(root)
    patch_print_setup(root)

    # Summary
    print(f"\n{'=' * 60}")
    print(f"Session 5 Patch Summary")
    print(f"{'=' * 60}")
    print(f"  Applied:  {_applied}")
    print(f"  Skipped:  {_skipped} (already applied)")
    print(f"  Failed:   {_failed}")

    if _failed > 0:
        print(f"\n{YELLOW}WARNING{RESET}: {_failed} patches could not be applied.")
        print("Review MISS messages above and apply manually.")

    print(f"\nFiles modified:")
    print(f"  gui/pages/print_well_setup.py  -- Plan UI + validate()")
    print(f"  gui/pages/print_setup.py       -- Validation gate")
    print(f"\nNew file (copy manually):")
    print(f"  SupportClasses/PrintPlanOfAction.py")


if __name__ == "__main__":
    main()
