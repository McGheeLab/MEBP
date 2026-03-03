#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 1 Patch
Config Load Pump Fix + XY Motion Unit Fix

Issues Covered: #1 (HW config load doesn't update pumps/needle-pump)
                #2 (XY jog moves wrong distance — Prior expects microns)

Files Modified:
    gui/pages/hardware_setup.py  — Fix _apply_config_to_ui pump restore
    gui/pages/jog_control.py     — Remove microstep conversion, send µm directly
    SupportClasses/StageController.py — Add move_xy_relative_um(), logging

Prerequisites:
    - MEBP v7.2.4 fully applied

Usage:
    python patch_s1_config_load_and_xy_fix.py [/path/to/MEBP]
"""

import os
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ═══════════════════════════════════════════════════════════════════
# Terminal Colors
# ═══════════════════════════════════════════════════════════════════
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
RESET  = "\033[0m"

_applied = 0
_skipped = 0
_failed  = 0


# ═══════════════════════════════════════════════════════════════════
# Utility Functions
# ═══════════════════════════════════════════════════════════════════

def find_project_root() -> Path:
    """Auto-detect the MEBP project root."""
    candidates = [
        Path.cwd(),
        Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
    print("Pass the project path as argument: python patch_s1_... /path/to/MEBP")
    sys.exit(1)


def backup_file(filepath: Path):
    """Create a timestamped backup."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = filepath.with_suffix(f".bak_v725s1_{ts}")
    shutil.copy2(filepath, backup)
    return backup


def read_file(filepath: Path) -> str:
    return filepath.read_text(encoding="utf-8")


def write_file(filepath: Path, content: str):
    filepath.write_text(content, encoding="utf-8")


def patch_replace(content: str, old: str, new: str, description: str, filepath: str = "") -> str:
    """Replace old text with new. Reports status."""
    global _applied, _skipped, _failed

    if new.strip() in content and old.strip() not in content:
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


def patch_replace_regex(content: str, pattern: str, replacement: str, description: str) -> str:
    """Replace using regex pattern. Reports status."""
    global _applied, _skipped, _failed

    match = re.search(pattern, content, re.DOTALL)
    if match is None:
        # Check if already applied
        if replacement.strip()[:60] in content:
            print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
            _skipped += 1
        else:
            print(f"  {RED}MISS{RESET}: {description} — pattern not found")
            _failed += 1
        return content

    content = content[:match.start()] + replacement + content[match.end():]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def insert_after(content: str, anchor: str, new_text: str, description: str) -> str:
    """Insert new_text after the anchor string."""
    global _applied, _skipped, _failed

    if new_text.strip()[:60] in content:
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


# ═══════════════════════════════════════════════════════════════════
# PATCH A: hardware_setup.py — Fix config load pump restore
# ═══════════════════════════════════════════════════════════════════

def patch_hardware_setup(root: Path):
    filepath = root / "gui" / "pages" / "hardware_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath.name} — Config Load Pump Fix")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)
    fp = str(filepath)

    # ── A1: Add _refresh_pump_ink_combos() call AFTER ink library restore ──
    # The issue is that _refresh_pump_ink_combos() is not called during restore,
    # so the pump widget combos may not have the ink names populated
    # Find the ink library restore step and add the combo refresh

    # In v7.2.4, step 3 is:
    #   self._refresh_ink_table()
    #   ink_names = list(self._config.ink_library.keys())
    # We need to add _refresh_pump_ink_combos() here

    old_ink_restore = (
        "        self._refresh_ink_table()\n"
        "        ink_names = list(self._config.ink_library.keys())\n"
        '        logger.debug(f"  Ink library: {len(ink_names)} inks")'
    )
    new_ink_restore = (
        "        self._refresh_ink_table()\n"
        "        # v7.2.5: Force pump ink combo refresh BEFORE pump restore\n"
        "        self._refresh_pump_ink_combos()\n"
        "        ink_names = list(self._config.ink_library.keys())\n"
        '        logger.debug(f"  Ink library: {len(ink_names)} inks — pump combos refreshed")'
    )
    content = patch_replace(content, old_ink_restore, new_ink_restore,
                            "A1: Add _refresh_pump_ink_combos() during ink library restore", fp)

    # ── A2: Add defensive logging in pump restore step ──
    # After pump config is applied, log whether the ink was successfully set

    old_pump_restore_end = (
        '        # Refresh exclusions after all pumps loaded\n'
        '        self._refresh_pump_ink_exclusions()\n'
        '        self._update_pump_ink_summary()'
    )
    new_pump_restore_end = (
        '        # v7.2.5: Verify pump ink assignments after restore\n'
        '        for pid, pw in self._pump_widgets.items():\n'
        '            actual_ink = pw.ink_combo.currentData() if hasattr(pw, "ink_combo") else None\n'
        '            expected_ink = self._config.pumps[pid].ink.name if (\n'
        '                pid in self._config.pumps and self._config.pumps[pid].ink) else None\n'
        '            if expected_ink and actual_ink != expected_ink:\n'
        '                logger.warning(\n'
        '                    f"  {pid} ink mismatch: expected={expected_ink}, "\n'
        '                    f"actual={actual_ink}. Re-applying...")\n'
        '                # Force re-apply: set the combo directly\n'
        '                idx = pw.ink_combo.findText(expected_ink)\n'
        '                if idx >= 0:\n'
        '                    pw.ink_combo.blockSignals(True)\n'
        '                    pw.ink_combo.setCurrentIndex(idx)\n'
        '                    pw.ink_combo.blockSignals(False)\n'
        '                else:\n'
        '                    logger.warning(f"  {pid}: ink {expected_ink!r} not in combo options")\n'
        '\n'
        '        # Refresh exclusions after all pumps loaded\n'
        '        self._refresh_pump_ink_exclusions()\n'
        '        self._update_pump_ink_summary()'
    )
    content = patch_replace(content, old_pump_restore_end, new_pump_restore_end,
                            "A2: Add pump ink verification + force re-apply after restore", fp)

    # ── A3: Fix channel map restore — add fallback logging ──
    old_channel_restore = (
        '            if mapped_pump:\n'
        '                idx = combo.findData(mapped_pump)\n'
        '                if idx >= 0:\n'
        '                    combo.blockSignals(True)\n'
        '                    combo.setCurrentIndex(idx)\n'
        '                    combo.blockSignals(False)'
    )
    new_channel_restore = (
        '            if mapped_pump:\n'
        '                idx = combo.findData(mapped_pump)\n'
        '                if idx >= 0:\n'
        '                    combo.blockSignals(True)\n'
        '                    combo.setCurrentIndex(idx)\n'
        '                    combo.blockSignals(False)\n'
        '                else:\n'
        '                    logger.warning(\n'
        '                        f"  Channel {ch_idx}: pump {mapped_pump!r} "\n'
        '                        f"not found in combo. Available: "\n'
        '                        f"{[combo.itemData(i) for i in range(combo.count())]}")'
    )
    content = patch_replace(content, old_channel_restore, new_channel_restore,
                            "A3: Add fallback logging for channel map combo misses", fp)

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH B: jog_control.py — Fix XY motion to send microns directly
# ═══════════════════════════════════════════════════════════════════

def patch_jog_control(root: Path):
    filepath = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath.name} — XY Motion Unit Fix")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)
    fp = str(filepath)

    # ── B1: Fix _jog_xy() — Send microns directly, not microsteps ──
    # The Prior ProScan controller expects microns directly.
    # Current code does: step_steps = step_um * self._microsteps_per_micron
    # Fix: send step_um directly as microns

    # Find the _jog_xy method body (v7.2.4 version with step tracking)
    old_jog_body = (
        '        if not self.controller.is_xy_connected:\n'
        '            return\n'
        '\n'
        '        # Get step size in microns from combo box\n'
        '        step_um = self.xy_step_combo.currentData()\n'
        '        # Convert µm → microsteps\n'
        '        step_steps = step_um * self._microsteps_per_micron\n'
        '\n'
        '        # v7.2.4: Record pre-jog position for verification display\n'
        '        self._last_xy_before = self.controller.get_xy_position(cached=True)\n'
        '        self._last_jog_step_um = step_um\n'
        '\n'
        '        # BUG-1 FIX: Use relative move — no dependency on cached position\n'
        '        self.controller.move_xy_relative(dx * step_steps, dy * step_steps)\n'
        '\n'
        '        # v7.2.4: Update step verification display\n'
        '        direction = ""\n'
        '        if dx > 0: direction = "X+"\n'
        '        elif dx < 0: direction = "X−"\n'
        '        if dy > 0: direction += "Y+"\n'
        '        elif dy < 0: direction += "Y−"\n'
        '        cmd_steps = round(step_steps)\n'
        "        if hasattr(self, '_lbl_last_jog'):\n"
        '            self._lbl_last_jog.setText(\n'
        '                f"Last: {direction} {step_um:g} µm "\n'
        '                f"({cmd_steps} steps)")\n'
        '        logger.debug(f"Jog {direction}: {step_um:g} µm = "\n'
        '                     f"{cmd_steps} microsteps "\n'
        '                     f"(factor={self._microsteps_per_micron})")'
    )
    new_jog_body = (
        '        if not self.controller.is_xy_connected:\n'
        '            return\n'
        '\n'
        '        # Get step size in microns from combo box\n'
        '        step_um = self.xy_step_combo.currentData()\n'
        '\n'
        '        # v7.2.5: Record pre-jog position for verification display\n'
        '        self._last_xy_before = self.controller.get_xy_position(cached=True)\n'
        '        self._last_jog_step_um = step_um\n'
        '\n'
        '        # v7.2.5 FIX: Send microns directly to controller.\n'
        '        # The Prior ProScan expects movement values in microns,\n'
        '        # NOT microsteps. The old code multiplied by microsteps_per_micron\n'
        '        # which caused incorrect movement distances.\n'
        '        self.controller.move_xy_relative_um(dx * step_um, dy * step_um)\n'
        '\n'
        '        # Update step verification display\n'
        '        direction = ""\n'
        '        if dx > 0: direction = "X+"\n'
        '        elif dx < 0: direction = "X−"\n'
        '        if dy > 0: direction += "Y+"\n'
        '        elif dy < 0: direction += "Y−"\n'
        "        if hasattr(self, '_lbl_last_jog'):\n"
        '            self._lbl_last_jog.setText(\n'
        '                f"Last: {direction} {step_um:g} µm")\n'
        '        logger.debug(f"Jog {direction}: {step_um:g} µm sent directly")'
    )
    content = patch_replace(content, old_jog_body, new_jog_body,
                            "B1: Fix _jog_xy — send microns directly, no microstep conversion", fp)

    # ── B1-alt: Handle case where v7.2.4 step tracking wasn't applied ──
    # (original v7.1.2 body without step tracking)
    if _failed > 0 or 'move_xy_relative_um' not in content:
        old_jog_simple = (
            '        if not self.controller.is_xy_connected:\n'
            '            return\n'
            '\n'
            '        # Get step size in microns from combo box\n'
            '        step_um = self.xy_step_combo.currentData()\n'
            '        # Convert µm → microsteps\n'
            '        step_steps = step_um * self._microsteps_per_micron\n'
            '\n'
            '        # BUG-1 FIX: Use relative move — no dependency on cached position\n'
            '        self.controller.move_xy_relative(dx * step_steps, dy * step_steps)'
        )
        new_jog_simple = (
            '        if not self.controller.is_xy_connected:\n'
            '            return\n'
            '\n'
            '        # Get step size in microns from combo box\n'
            '        step_um = self.xy_step_combo.currentData()\n'
            '\n'
            '        # v7.2.5 FIX: Send microns directly to controller.\n'
            '        # Prior ProScan expects microns, not microsteps.\n'
            '        self.controller.move_xy_relative_um(dx * step_um, dy * step_um)\n'
            '        logger.debug(f"Jog: {step_um:g} µm sent directly")'
        )
        content = patch_replace(content, old_jog_simple, new_jog_simple,
                                "B1-alt: Fix _jog_xy (pre-v7.2.4 version)", fp)

    # ── B2: Fix position readback — display microns directly ──
    # The controller returns positions in microns, so we should NOT
    # divide by microsteps_per_micron

    old_pos_xy = (
        '        # XY position (convert steps → µm)\n'
        '        xy = ctrl.get_xy_position(cached=True)\n'
        '        if xy[0] is not None:\n'
        '            zx = xy[0] - ctrl.zero_position["x"]\n'
        '            zy = xy[1] - ctrl.zero_position["y"]\n'
        '            ux = zx / self._microsteps_per_micron\n'
        '            uy = zy / self._microsteps_per_micron\n'
        '            self.lbl_x.setText(f"{ux:,.1f}")\n'
        '            self.lbl_y.setText(f"{uy:,.1f}")'
    )
    new_pos_xy = (
        '        # XY position — v7.2.5: controller reports in microns directly\n'
        '        xy = ctrl.get_xy_position(cached=True)\n'
        '        if xy[0] is not None:\n'
        '            ux = xy[0] - ctrl.zero_position["x"]\n'
        '            uy = xy[1] - ctrl.zero_position["y"]\n'
        '            self.lbl_x.setText(f"{ux:,.1f}")\n'
        '            self.lbl_y.setText(f"{uy:,.1f}")'
    )
    content = patch_replace(content, old_pos_xy, new_pos_xy,
                            "B2: Fix XY position readback — display microns directly", fp)

    # ── B3: Fix step verification delta display (if present) ──
    # v7.2.4 added delta computation that also divided by microsteps_per_micron
    old_verify = (
        '            dx_steps = abs(xy[0] - self._last_xy_before[0]) + \\\n'
        '                       abs(xy[1] - self._last_xy_before[1])\n'
        '            dx_um = dx_steps / self._microsteps_per_micron'
    )
    new_verify = (
        '            # v7.2.5: positions are already in microns\n'
        '            dx_um = abs(xy[0] - self._last_xy_before[0]) + \\\n'
        '                    abs(xy[1] - self._last_xy_before[1])'
    )
    content = patch_replace(content, old_verify, new_verify,
                            "B3: Fix step verification — positions already in microns", fp)

    # ── B4: Update _jog_xy_home to use micron-based method ──
    old_home = (
        '    def _jog_xy_home(self):\n'
        '        """Move to the zero reference position (absolute move, this is fine)."""\n'
        '        if self.controller.is_xy_connected:\n'
        '            self.controller.move_xy_absolute(0, 0, from_zero_ref=True)'
    )
    # Home move at 0,0 relative to zero is fine — the absolute method already handles
    # zero offset in StageController. No change needed here, but let's add a note.
    # Actually, this is fine as-is since move_xy_absolute works in zero-relative coords
    # and the controller handles the conversion. Keep as-is.

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH C: StageController.py — Add move_xy_relative_um()
# ═══════════════════════════════════════════════════════════════════

def patch_stage_controller(root: Path):
    filepath = root / "SupportClasses" / "StageController.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH C: {filepath.name} — Add move_xy_relative_um()")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)
    fp = str(filepath)

    # ── C1: Add move_xy_relative_um() method ──
    # Insert after the existing move_xy_relative method

    new_method = '''
    def move_xy_relative_um(self, dx_um: float, dy_um: float) -> None:
        """
        Move XY stage by a relative offset in MICRONS.

        v7.2.5: The Prior ProScan controller accepts movement commands
        in microns directly. This method sends micron values without
        any microstep conversion.

        Safety limits are projected from cached position.
        """
        if not self.xy_stage:
            return

        # Safety: project cached position + delta, clamp if needed
        # Note: safety limits and positions are all in the same units (microns)
        if self.safety_limits.enabled:
            pos = self.get_xy_position(cached=True)
            if pos[0] is not None:
                zero_x = self.zero_position["x"]
                zero_y = self.zero_position["y"]
                cur_zr_x = pos[0] - zero_x
                cur_zr_y = pos[1] - zero_y
                new_zr_x = cur_zr_x + dx_um
                new_zr_y = cur_zr_y + dy_um
                clamped_x, clamped_y = self.safety_limits.clamp_xy(new_zr_x, new_zr_y)
                dx_um = clamped_x - cur_zr_x
                dy_um = clamped_y - cur_zr_y

        logger.debug(f"move_xy_relative_um: sending dx={dx_um:.1f} dy={dy_um:.1f} µm")
        self.xy_stage.move_stage_relative(dx_um, dy_um)

'''

    # Find the end of move_xy_relative method
    anchor = '        self.xy_stage.move_stage_relative(dx, dy)\n'

    # Check for v7.2.4 version with logging
    anchor_v724 = (
        "        logger.debug(f\"move_xy_relative: sending dx={round(dx)} dy={round(dy)} \"\n"
        "                     f\"microsteps (raw: dx={dx:.2f} dy={dy:.2f})\")\n"
        "        self.xy_stage.move_stage_relative(dx, dy)\n"
    )

    if 'move_xy_relative_um' in content:
        print(f"  {YELLOW}SKIP{RESET}: C1 — move_xy_relative_um already exists")
        global _skipped
        _skipped += 1
    elif anchor_v724 in content:
        content = insert_after(content, anchor_v724, new_method,
                              "C1: Add move_xy_relative_um() after move_xy_relative (v7.2.4)")
    elif anchor in content:
        # Find the LAST occurrence of this anchor (inside move_xy_relative)
        # We need to insert after the one inside move_xy_relative, not move_xy_absolute
        idx = content.find('def move_xy_relative(self, dx: float, dy: float)')
        if idx >= 0:
            # Find the anchor after the method definition
            anchor_pos = content.find(anchor, idx)
            if anchor_pos >= 0:
                insert_pos = anchor_pos + len(anchor)
                content = content[:insert_pos] + new_method + content[insert_pos:]
                print(f"  {GREEN}OK{RESET}:   C1: Add move_xy_relative_um() after move_xy_relative")
                global _applied
                _applied += 1
            else:
                print(f"  {RED}MISS{RESET}: C1 — anchor not found after method definition")
                global _failed
                _failed += 1
        else:
            content = insert_after(content, anchor, new_method,
                                  "C1: Add move_xy_relative_um() (fallback)")
    else:
        print(f"  {RED}MISS{RESET}: C1 — could not find move_xy_relative anchor")
        _failed += 1

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH D: app.py — Simplify microstep propagation
# ═══════════════════════════════════════════════════════════════════

def patch_app(root: Path):
    filepath = root / "gui" / "app.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH D: {filepath.name} — Add note about micron-direct mode")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)
    fp = str(filepath)

    # ── D1: Add a comment in the microsteps propagation section ──
    # We don't want to break existing functionality — just add a note
    # that the jog page now uses microns directly

    old_propagate = "            if hasattr(page, 'set_microsteps_per_micron'):"
    new_propagate = (
        "            # v7.2.5 NOTE: Jog page now sends microns directly to the\n"
        "            # Prior controller via move_xy_relative_um(). The microsteps_per_micron\n"
        "            # factor is kept for backward compat with other pages that may need it.\n"
        "            if hasattr(page, 'set_microsteps_per_micron'):"
    )
    content = patch_replace(content, old_propagate, new_propagate,
                            "D1: Add note about micron-direct mode in propagation", fp)

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
    print(f" MEBP v7.2.5 — Session 1 Patch")
    print(f" Config Load Pump Fix + XY Motion Unit Fix")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    # Verify prerequisites
    hw_setup = root / "gui" / "pages" / "hardware_setup.py"
    jog_ctrl = root / "gui" / "pages" / "jog_control.py"
    stage_ctrl = root / "SupportClasses" / "StageController.py"
    app_py = root / "gui" / "app.py"

    missing = []
    for f in [hw_setup, jog_ctrl, stage_ctrl, app_py]:
        if not f.exists():
            missing.append(str(f.relative_to(root)))
    if missing:
        print(f"\n{RED}ERROR{RESET}: Missing files:")
        for m in missing:
            print(f"  - {m}")
        sys.exit(1)

    # Apply patches
    patch_hardware_setup(root)
    patch_jog_control(root)
    patch_stage_controller(root)
    patch_app(root)

    # Summary
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'═' * 60}")
    print(f" SUMMARY")
    print(f"{'═' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}: {_applied}")
    print(f"  {YELLOW}Skipped{RESET}: {_skipped} (already applied)")
    print(f"  {RED}Failed{RESET}:  {_failed}")
    print(f"  Total:   {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} patch(es) failed!")
        print("Check the MISS messages above and apply manually if needed.")
        print("This may indicate the source files have been modified from")
        print("the expected v7.2.4 state.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All patches applied successfully!{RESET}")


if __name__ == "__main__":
    main()
