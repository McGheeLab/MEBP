#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 2 Patch
Pump µL Controls + Generate Print Button

Issues Covered: #3 (Pump inactive if no pump; movements in µL)
                #4 (Print Settings "Generate Print" button)

Files Modified:
    gui/pages/jog_control.py  — Pump enable/disable, µL step sizes + display
    gui/pages/print_setup.py  — Add Generate Print button

Prerequisites:
    - Session 1 patch applied

Usage:
    python patch_s2_pump_ul_and_generate.py [/path/to/MEBP]
"""

import os
import re
import sys

# === Windows console encoding fix ===
import sys as _sys
if _sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(_sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')
# === End encoding fix ===

import shutil
from pathlib import Path
from datetime import datetime

# Terminal colors
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
RESET  = "\033[0m"

_applied = 0
_skipped = 0
_failed  = 0


def find_project_root() -> Path:
    candidates = [
        Path.cwd(), Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
    sys.exit(1)


def backup_file(filepath: Path):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = filepath.with_suffix(f".bak_v725s2_{ts}")
    shutil.copy2(filepath, backup)


def read_file(filepath: Path) -> str:
    return filepath.read_text(encoding="utf-8")


def write_file(filepath: Path, content: str):
    filepath.write_text(content, encoding="utf-8")


def patch_replace(content: str, old: str, new: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new.strip()[:80] in content and old not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if old not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    content = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def insert_after(content: str, anchor: str, new_text: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new_text.strip()[:80] in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    idx = content.find(anchor) + len(anchor)
    content = content[:idx] + new_text + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def insert_before(content: str, anchor: str, new_text: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new_text.strip()[:80] in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    idx = content.find(anchor)
    content = content[:idx] + new_text + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH A: jog_control.py — Pump µL controls
# ═══════════════════════════════════════════════════════════════════

def patch_jog_control(root: Path):
    filepath = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath.name} — Pump µL Controls")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── A1: Store pump button references for enable/disable ──
    # Replace the pump button creation loop to store references

    old_pump_loop = (
        '        # Pump buttons\n'
        '        for pump_name in ["P1", "P2", "P3"]:\n'
        '            p_group = QVBoxLayout()\n'
        '            p_group.addWidget(QLabel(pump_name))\n'
        '            btn_ext = QPushButton(f"{pump_name} ▲")\n'
        '            btn_ext.clicked.connect(partial(self._jog_pump, pump_name, 1))\n'
        '            p_group.addWidget(btn_ext)\n'
        '            btn_ret = QPushButton(f"{pump_name} ▼")\n'
        '            btn_ret.clicked.connect(partial(self._jog_pump, pump_name, -1))\n'
        '            p_group.addWidget(btn_ret)\n'
        '            zp_layout.addLayout(p_group)'
    )
    new_pump_loop = (
        '        # Pump buttons — v7.2.5: store refs for enable/disable\n'
        '        self._pump_buttons = {}  # {pump_name: (label, btn_ext, btn_ret)}\n'
        '        self._pump_labels = {}   # {pump_name: QLabel for position}\n'
        '        for pump_name in ["P1", "P2", "P3"]:\n'
        '            p_group = QVBoxLayout()\n'
        '            p_label = QLabel(pump_name)\n'
        '            p_group.addWidget(p_label)\n'
        '            btn_ext = QPushButton(f"{pump_name} ▲")\n'
        '            btn_ext.clicked.connect(partial(self._jog_pump, pump_name, 1))\n'
        '            p_group.addWidget(btn_ext)\n'
        '            btn_ret = QPushButton(f"{pump_name} ▼")\n'
        '            btn_ret.clicked.connect(partial(self._jog_pump, pump_name, -1))\n'
        '            p_group.addWidget(btn_ret)\n'
        '            zp_layout.addLayout(p_group)\n'
        '            self._pump_buttons[pump_name] = (p_label, btn_ext, btn_ret)'
    )
    content = patch_replace(content, old_pump_loop, new_pump_loop,
                            "A1: Store pump button refs for enable/disable")

    # ── A2: Add _hardware_config attribute to __init__ if not present ──
    if '_hardware_config = None' not in content and '_hardware_config' not in content.split('def _setup_ui')[0]:
        content = insert_after(
            content,
            '        self._context_widget = None',
            '\n\n        # v7.2.5: Hardware config for pump enable/disable and µL display\n'
            '        self._hardware_config = None',
            "A2: Add _hardware_config attribute to __init__"
        )

    # ── A3: Enhance set_hardware_config to enable/disable pump buttons ──
    old_set_hw = (
        '    def set_hardware_config(self, config):\n'
        '        """v7.2.4: Receive hardware config (for future pump µL display)."""\n'
        '        self._hardware_config = config'
    )
    new_set_hw = (
        '    def set_hardware_config(self, config):\n'
        '        """v7.2.5: Receive hardware config — enable/disable pump buttons, µL display."""\n'
        '        self._hardware_config = config\n'
        '        self._update_pump_states()'
    )
    content = patch_replace(content, old_set_hw, new_set_hw,
                            "A3: Enhance set_hardware_config for pump state updates")

    # ── A4: Add _update_pump_states method ──
    update_pump_method = '''
    def _update_pump_states(self):
        """v7.2.5: Enable/disable pump buttons based on HardwareConfig."""
        if not hasattr(self, '_pump_buttons'):
            return

        configured_pumps = set()
        if self._hardware_config:
            configured_pumps = set(self._hardware_config.configured_pump_ids)

        for pump_name, (label, btn_ext, btn_ret) in self._pump_buttons.items():
            is_configured = pump_name in configured_pumps
            btn_ext.setEnabled(is_configured)
            btn_ret.setEnabled(is_configured)
            if is_configured:
                label.setStyleSheet(f"color: {COLORS['text']};")
                btn_ext.setToolTip(f"Extend {pump_name}")
                btn_ret.setToolTip(f"Retract {pump_name}")
            else:
                label.setStyleSheet(f"color: {COLORS.get('overlay0', '#6c7086')};")
                btn_ext.setToolTip(f"{pump_name} not configured")
                btn_ret.setToolTip(f"{pump_name} not configured")

        # Update pump position labels in readout
        for col, pump_name in enumerate(["P1", "P2", "P3"], start=3):
            lbl_attr = f"lbl_p{pump_name[-1]}"
            if hasattr(self, lbl_attr):
                lbl = getattr(self, lbl_attr)
                if pump_name not in configured_pumps:
                    lbl.setText("—")
                    lbl.setStyleSheet(
                        f"color: {COLORS.get('overlay0', '#6c7086')}; font-size: 11px;")

'''
    content = insert_after(
        content,
        '        self._hardware_config = config\n'
        '        self._update_pump_states()',
        update_pump_method,
        "A4: Add _update_pump_states method"
    )

    # ── A5: Modify _jog_pump to convert µL to mm ──
    old_jog_pump = (
        '    def _jog_pump(self, pump: str, direction: int):\n'
        '        if self.controller.is_zp_connected:\n'
        '            self.controller.move_pump_relative(pump, direction * self.p_step_combo.currentData())'
    )
    new_jog_pump = (
        '    def _jog_pump(self, pump: str, direction: int):\n'
        '        """v7.2.5: Jog pump — convert µL step to mm if HW config available."""\n'
        '        if not self.controller.is_zp_connected:\n'
        '            return\n'
        '        step_val = self.p_step_combo.currentData()\n'
        '        # v7.2.5: If HW config available, step_val is in µL — convert to mm\n'
        '        if self._hardware_config:\n'
        '            pump_cfg = self._hardware_config.pumps.get(pump)\n'
        '            if pump_cfg and pump_cfg.is_configured:\n'
        '                try:\n'
        '                    step_mm = self._hardware_config.uL_to_mm(pump, step_val)\n'
        '                    self.controller.move_pump_relative(pump, direction * step_mm)\n'
        '                    logger.debug(f"Pump {pump}: {step_val} µL = {step_mm:.4f} mm")\n'
        '                    return\n'
        '                except (ValueError, AttributeError) as e:\n'
        '                    logger.warning(f"µL→mm conversion failed for {pump}: {e}")\n'
        '        # Fallback: use raw value as mm\n'
        '        self.controller.move_pump_relative(pump, direction * step_val)'
    )
    content = patch_replace(content, old_jog_pump, new_jog_pump,
                            "A5: Modify _jog_pump for µL→mm conversion")

    # ── A6: Change pump step combo to µL values ──
    old_p_steps = "P_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]"
    new_p_steps = (
        "# v7.2.5: Pump steps in µL (converted to mm via HardwareConfig)\n"
        "# Fallback: used as mm if no HW config available\n"
        "P_STEPS_UL = [0.1, 0.5, 1.0, 5.0, 10.0, 50.0]\n"
        "P_STEPS  = [0.01, 0.05, 0.1, 0.5, 1.0, 5.0]  # legacy mm fallback"
    )
    content = patch_replace(content, old_p_steps, new_p_steps,
                            "A6: Add µL step sizes constant")

    # ── A7: Update pump step combo population ──
    # The context panel builds the pump step combo. We need to find it and
    # add µL options when hardware config is available.
    # Since the combo is built in get_context_widget, we'll add a refresh method.

    pump_step_refresh = '''
    def _refresh_pump_step_combo(self):
        """v7.2.5: Update pump step combo with µL values if HW config available."""
        if not hasattr(self, 'p_step_combo'):
            return
        self.p_step_combo.blockSignals(True)
        current_data = self.p_step_combo.currentData()
        self.p_step_combo.clear()
        if self._hardware_config and self._hardware_config.configured_pump_ids:
            # Use µL step sizes
            for s in P_STEPS_UL:
                self.p_step_combo.addItem(f"{s:g} µL", s)
            # Try to restore selection
            idx = self.p_step_combo.findData(current_data)
            if idx >= 0:
                self.p_step_combo.setCurrentIndex(idx)
            else:
                self.p_step_combo.setCurrentIndex(2)  # Default 1.0 µL
        else:
            # Fallback: mm step sizes
            for s in P_STEPS:
                self.p_step_combo.addItem(f"{s:g} mm", s)
            idx = self.p_step_combo.findData(current_data)
            if idx >= 0:
                self.p_step_combo.setCurrentIndex(idx)
            else:
                self.p_step_combo.setCurrentIndex(2)  # Default 0.1 mm
        self.p_step_combo.blockSignals(False)

'''
    # Insert before _setup_shortcuts
    content = insert_before(
        content,
        '    def _setup_shortcuts(self):',
        pump_step_refresh,
        "A7: Add _refresh_pump_step_combo method"
    )

    # ── A8: Call refresh from set_hardware_config ──
    content = patch_replace(
        content,
        '        self._update_pump_states()',
        '        self._update_pump_states()\n'
        '        self._refresh_pump_step_combo()',
        "A8: Call _refresh_pump_step_combo from set_hardware_config"
    )

    # ── A9: Update pump position display to show µL ──
    old_pump_display = (
        '        # ZP position (already in mm)\n'
        '        zp = ctrl.get_zp_position(cached=True)\n'
        '        if zp[0] is not None:\n'
        '            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position[\'Z\']:.2f}")\n'
        '            self.lbl_p1.setText(f"{zp[1] - ctrl.zero_position[\'P1\']:.2f}")\n'
        '            self.lbl_p2.setText(f"{zp[2] - ctrl.zero_position[\'P2\']:.2f}")\n'
        '            self.lbl_p3.setText(f"{zp[3] - ctrl.zero_position[\'P3\']:.2f}")\n'
        '        else:\n'
        '            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:\n'
        '                lbl.setText("—")'
    )
    new_pump_display = (
        '        # ZP position\n'
        '        zp = ctrl.get_zp_position(cached=True)\n'
        '        if zp[0] is not None:\n'
        '            self.lbl_z.setText(f"{zp[0] - ctrl.zero_position[\'Z\']:.2f}")\n'
        '            # v7.2.5: Display pump positions in µL if HW config available\n'
        '            for pidx, pid in enumerate(["P1", "P2", "P3"], start=1):\n'
        '                lbl = getattr(self, f"lbl_p{pidx}", None)\n'
        '                if lbl is None:\n'
        '                    continue\n'
        '                pos_mm = zp[pidx] - ctrl.zero_position[pid]\n'
        '                if (self._hardware_config and\n'
        '                        pid in self._hardware_config.configured_pump_ids):\n'
        '                    try:\n'
        '                        pos_uL = self._hardware_config.mm_to_uL(pid, pos_mm)\n'
        '                        lbl.setText(f"{pos_uL:.2f}")\n'
        '                        continue\n'
        '                    except (ValueError, AttributeError):\n'
        '                        pass\n'
        '                lbl.setText(f"{pos_mm:.2f}")\n'
        '        else:\n'
        '            for lbl in [self.lbl_z, self.lbl_p1, self.lbl_p2, self.lbl_p3]:\n'
        '                lbl.setText("—")'
    )
    content = patch_replace(content, old_pump_display, new_pump_display,
                            "A9: Update pump position display to show µL")

    # ── A10: Update position header labels to show µL when configured ──
    old_pos_header = (
        '            ("P1:", "lbl_p1"), ("P2:", "lbl_p2"), ("P3:", "lbl_p3"),'
    )
    new_pos_header = (
        '            ("P1 (µL):", "lbl_p1"), ("P2 (µL):", "lbl_p2"), ("P3 (µL):", "lbl_p3"),'
    )
    content = patch_replace(content, old_pos_header, new_pos_header,
                            "A10: Update position headers to show µL units")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH B: print_setup.py — Generate Print button
# ═══════════════════════════════════════════════════════════════════

def patch_print_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath.name} — Generate Print Button")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── B1: Add "Generate Print" button BEFORE "Send to Monitor" ──
    old_send_section = (
        "        self.btn_send_to_monitor = QPushButton(\"📤 Send to Monitor ▶\")"
    )
    new_generate_section = (
        '        # v7.2.5: Generate Print button — validate + build execution plan\n'
        '        self.btn_generate_print = QPushButton("⚙ Generate Print")\n'
        '        self.btn_generate_print.setStyleSheet(f"""\n'
        '            QPushButton {{\n'
        '                background: {COLORS.get(\'blue\', \'#89b4fa\')};\n'
        '                color: {COLORS.get(\'base\', \'#1e1e2e\')};\n'
        '                font-weight: bold; padding: 8px 16px;\n'
        '                border-radius: 4px;\n'
        '            }}\n'
        '            QPushButton:hover {{\n'
        '                background: {COLORS.get(\'sapphire\', \'#74c7ec\')};\n'
        '            }}\n'
        '        """)\n'
        '        self.btn_generate_print.setToolTip(\n'
        '            "Validate well setup and generate execution plan")\n'
        '        self.btn_generate_print.clicked.connect(self._generate_print)\n'
        '        layout.addWidget(self.btn_generate_print)\n'
        '\n'
        '        # Generation status label\n'
        '        self._gen_status_label = QLabel("")\n'
        '        self._gen_status_label.setStyleSheet(\n'
        '            f"color: {COLORS.get(\'subtext0\', \'#a6adc8\')}; font-size: 10px;")\n'
        '        self._gen_status_label.setWordWrap(True)\n'
        '        layout.addWidget(self._gen_status_label)\n'
        '\n'
        "        self.btn_send_to_monitor = QPushButton(\"📤 Send to Monitor ▶\")"
    )
    content = patch_replace(content, old_send_section, new_generate_section,
                            "B1: Add Generate Print button before Send to Monitor")

    # ── B2: Add _generate_print method ──
    generate_method = '''
    def _generate_print(self):
        """
        v7.2.5: Validate well setup and generate execution plan.

        This is the primary "prepare for print" action. It:
        1. Validates the well setup (roles, inks, service wells)
        2. Generates the execution plan (runs, service steps)
        3. Shows results in the status label
        4. Enables/disables Send to Monitor based on validity
        """
        # Step 1: Validate
        if hasattr(self.tab_wells, 'validate'):
            is_valid, issues = self.tab_wells.validate()
        else:
            is_valid, issues = False, ["Well setup tab missing validate() method"]

        if not is_valid:
            self._gen_status_label.setText(
                f"⚠ {len(issues)} issue(s):\\n" +
                "\\n".join(f"  • {i}" for i in issues[:5]))
            self._gen_status_label.setStyleSheet(
                f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
            self.btn_send_to_monitor.setEnabled(False)
            logger.warning(f"Generate Print: {len(issues)} validation issues")
            return

        # Step 2: Generate plan
        if hasattr(self.tab_wells, '_generate_plan'):
            try:
                self.tab_wells._generate_plan()
            except Exception as e:
                self._gen_status_label.setText(f"⚠ Plan generation failed: {e}")
                self._gen_status_label.setStyleSheet(
                    f"color: {COLORS.get('red', '#f38ba8')}; font-size: 10px;")
                self.btn_send_to_monitor.setEnabled(False)
                return

        # Step 3: Show success
        plan = self.tab_wells.get_plan() if hasattr(self.tab_wells, 'get_plan') else None
        summary_parts = ["✓ Print plan generated"]
        if plan:
            summary_parts.append(f"{plan.total_runs} run(s)")
            if hasattr(plan, 'estimated_total_seconds') and plan.estimated_total_seconds > 0:
                summary_parts.append(f"~{plan.estimated_total_seconds / 60:.1f} min")
        self._gen_status_label.setText(" | ".join(summary_parts))
        self._gen_status_label.setStyleSheet(
            f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 10px;")
        self.btn_send_to_monitor.setEnabled(True)
        logger.info("Generate Print: plan ready")

'''
    # Insert before _export_gcode
    content = insert_before(
        content,
        '    def _export_gcode(self):',
        generate_method,
        "B2: Add _generate_print method"
    )

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'=' * 60}")
    print(f" MEBP v7.2.5 — Session 2 Patch")
    print(f" Pump µL Controls + Generate Print Button")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    patch_jog_control(root)
    patch_print_setup(root)

    # Summary
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'═' * 60}")
    print(f" SUMMARY")
    print(f"{'═' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}: {_applied}")
    print(f"  {YELLOW}Skipped{RESET}: {_skipped}")
    print(f"  {RED}Failed{RESET}:  {_failed}")
    print(f"  Total:   {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} patch(es) failed!")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All patches applied successfully!{RESET}")


if __name__ == "__main__":
    main()
