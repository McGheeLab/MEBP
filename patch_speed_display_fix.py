#!/usr/bin/env python3
"""
patch_speed_display_fix.py — Fix two runtime bugs causing console spam.

BUG A: StageController.get_speed_info() uses .speed and .speeds properties
       on XYJogHandler/ZPJogHandler, but the @property decorators were
       dropped by v7.2.6 patches, so they return method objects instead
       of values. Fix: read .xy_speed, .z_speed, .p_speed attributes directly.

BUG B: dashboard.py formats speed values with f-string :.0f, which crashes
       when the value is a method object (from Bug A). Fix: wrap in
       try/except with safe fallback, and use float() coercion.

Usage:
    python patch_speed_display_fix.py
    python patch_speed_display_fix.py /path/to/MEBP
"""

from __future__ import annotations

import os
import re
import sys
import ast
import shutil
from datetime import datetime

# ── Terminal colors ──────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

_applied = 0
_skipped = 0
_failed  = 0

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")


def find_project_root(hint: str | None = None) -> str:
    """Locate MEBP project root."""
    if hint and os.path.isdir(hint):
        return hint
    # Check common locations
    candidates = [
        os.getcwd(),
        os.path.join(os.getcwd(), "MEBP"),
        os.path.expanduser("~/Documents/GitHub/MEBP"),
    ]
    for c in candidates:
        if os.path.isfile(os.path.join(c, "main.py")) and os.path.isdir(
            os.path.join(c, "SupportClasses")
        ):
            return c
    print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
    print("Usage: python patch_speed_display_fix.py /path/to/MEBP")
    sys.exit(1)


def backup(path: str) -> None:
    """Create timestamped backup."""
    bak = f"{path}.bak_speedfix_{TIMESTAMP}"
    shutil.copy2(path, bak)


def ast_check(path: str) -> bool:
    """Verify file has valid Python syntax."""
    try:
        with open(path) as f:
            ast.parse(f.read())
        return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL{RESET}: {e}")
        return False


def read(path: str) -> str:
    with open(path) as f:
        return f.read()


def write(path: str, content: str) -> None:
    with open(path, "w") as f:
        f.write(content)


# ══════════════════════════════════════════════════════════════════
#  FIX A: StageController.get_speed_info()
# ══════════════════════════════════════════════════════════════════

def fix_get_speed_info(root: str) -> None:
    global _applied, _skipped, _failed
    path = os.path.join(root, "SupportClasses", "StageController.py")
    if not os.path.isfile(path):
        print(f"  {RED}MISS{RESET}: StageController.py not found")
        _failed += 1
        return

    content = read(path)
    backup(path)

    # ── A1: Fix get_speed_info to use direct attributes ──────────
    # The current broken code uses .speed and .speeds["z"] which fail
    # when @property decorators are missing. Replace with direct attrs.

    # Check if already fixed (idempotency)
    if 'getattr(self.xy_jog, "xy_speed"' in content or "self.xy_jog.xy_speed if self.xy_jog" in content:
        print(f"  {YELLOW}SKIP{RESET}: A1 — get_speed_info already uses direct attributes")
        _skipped += 1
    else:
        # Find the get_speed_info method using regex
        # Match the method signature and its return statement block
        pattern = re.compile(
            r'(    def get_speed_info\(self\)[^:]*:\s*\n)'  # method signature
            r'((?:        .*\n)*?)'                          # body lines
            r'(        \})\s*\n',                            # closing brace
            re.MULTILINE,
        )

        match = pattern.search(content)
        if match:
            new_method = (
                '    def get_speed_info(self) -> dict:\n'
                '        """Return current jog speeds as numeric values.\n'
                '\n'
                '        v7.2.8: Reads .xy_speed/.z_speed/.p_speed directly\n'
                '        to avoid broken @property decorators.\n'
                '        """\n'
                '        return {\n'
                '            "xy": getattr(self.xy_jog, "xy_speed", 0) if self.xy_jog else 0,\n'
                '            "z": getattr(self.zp_jog, "z_speed", 0) if self.zp_jog else 0,\n'
                '            "p": getattr(self.zp_jog, "p_speed", 0) if self.zp_jog else 0,\n'
                '        }\n'
            )
            content = content[:match.start()] + new_method + content[match.end():]
            print(f"  {GREEN}OK{RESET}: A1 — get_speed_info uses direct attributes")
            _applied += 1
        else:
            # Fallback: try simpler line-by-line replacement
            old_lines = [
                '"xy": self.xy_jog.speed if self.xy_jog else 0,',
                '"z": self.zp_jog.speeds["z"] if self.zp_jog else 0,',
                '"p": self.zp_jog.speeds["p"] if self.zp_jog else 0,',
            ]
            new_lines = [
                '"xy": getattr(self.xy_jog, "xy_speed", 0) if self.xy_jog else 0,',
                '"z": getattr(self.zp_jog, "z_speed", 0) if self.zp_jog else 0,',
                '"p": getattr(self.zp_jog, "p_speed", 0) if self.zp_jog else 0,',
            ]
            any_replaced = False
            for old, new in zip(old_lines, new_lines):
                if old in content:
                    content = content.replace(old, new, 1)
                    any_replaced = True

            if any_replaced:
                print(f"  {GREEN}OK{RESET}: A1 — get_speed_info fixed (line-by-line)")
                _applied += 1
            else:
                print(f"  {RED}MISS{RESET}: A1 — could not find get_speed_info body")
                print(f"         Look for 'def get_speed_info' in StageController.py")
                print(f"         and replace .speed/.speeds with .xy_speed/.z_speed/.p_speed")
                _failed += 1

    # Write and verify
    write(path, content)
    if not ast_check(path):
        print(f"  {RED}FATAL{RESET}: StageController.py AST broken after patch!")
        _failed += 1


# ══════════════════════════════════════════════════════════════════
#  FIX B: dashboard.py safe speed formatting
# ══════════════════════════════════════════════════════════════════

def fix_dashboard_speeds(root: str) -> None:
    global _applied, _skipped, _failed
    path = os.path.join(root, "gui", "pages", "dashboard.py")
    if not os.path.isfile(path):
        print(f"  {RED}MISS{RESET}: dashboard.py not found")
        _failed += 1
        return

    content = read(path)
    backup(path)

    # ── B1: Wrap speed display in try/except ─────────────────────
    # Old (fragile):
    #   speeds = ctrl.get_speed_info()
    #   self.lbl_speed_xy.setText(f"{speeds['xy']:.0f}")
    #   self.lbl_speed_z.setText(f"{speeds['z']:.1f}")
    #   self.lbl_speed_p.setText(f"{speeds['p']:.1f}")
    #
    # New (safe):
    #   try:
    #       speeds = ctrl.get_speed_info()
    #       self.lbl_speed_xy.setText(f"{float(speeds.get('xy', 0)):.0f}")
    #       ...

    # Check idempotency
    if "float(speeds.get('xy'" in content or "float(speeds.get(\"xy\"" in content:
        print(f"  {YELLOW}SKIP{RESET}: B1 — dashboard speed display already safe")
        _skipped += 1
    else:
        # Find the speed display block using regex
        # Pattern: speeds = ctrl.get_speed_info() followed by the three setText lines
        pattern = re.compile(
            r'([ \t]+)'                                          # capture indent
            r'speeds = ctrl\.get_speed_info\(\)\s*\n'
            r'\1self\.lbl_speed_xy\.setText\([^\n]+\)\s*\n'
            r'\1self\.lbl_speed_z\.setText\([^\n]+\)\s*\n'
            r'\1self\.lbl_speed_p\.setText\([^\n]+\)\s*\n',
        )
        match = pattern.search(content)
        if match:
            indent = match.group(1)
            lines = [
                indent + "try:\n",
                indent + "    speeds = ctrl.get_speed_info()\n",
                indent + "    self.lbl_speed_xy.setText(f\"{float(speeds.get('xy', 0)):.0f}\")\n",
                indent + "    self.lbl_speed_z.setText(f\"{float(speeds.get('z', 0)):.1f}\")\n",
                indent + "    self.lbl_speed_p.setText(f\"{float(speeds.get('p', 0)):.1f}\")\n",
                indent + "except (TypeError, ValueError, AttributeError):\n",
                indent + "    pass  # Speed info temporarily unavailable\n",
            ]
            replacement = "".join(lines)
            content = content[:match.start()] + replacement + content[match.end():]
            print(f"  {GREEN}OK{RESET}: B1 — dashboard speed display wrapped safely")
            _applied += 1
        else:
            # Try simpler individual line replacement
            replacements = [
                (
                    'self.lbl_speed_xy.setText(f"{speeds[\'xy\']:.0f}")',
                    'self.lbl_speed_xy.setText(f"{float(speeds.get(\'xy\', 0)):.0f}")',
                ),
                (
                    'self.lbl_speed_z.setText(f"{speeds[\'z\']:.1f}")',
                    'self.lbl_speed_z.setText(f"{float(speeds.get(\'z\', 0)):.1f}")',
                ),
                (
                    'self.lbl_speed_p.setText(f"{speeds[\'p\']:.1f}")',
                    'self.lbl_speed_p.setText(f"{float(speeds.get(\'p\', 0)):.1f}")',
                ),
            ]
            any_replaced = False
            for old, new in replacements:
                if old in content:
                    content = content.replace(old, new, 1)
                    any_replaced = True

            if any_replaced:
                print(f"  {GREEN}OK{RESET}: B1 — dashboard speed lines made safe (individual)")
                _applied += 1
            else:
                # Try with double quotes
                replacements_dq = [
                    (
                        "self.lbl_speed_xy.setText(f\"{speeds['xy']:.0f}\")",
                        "self.lbl_speed_xy.setText(f\"{float(speeds.get('xy', 0)):.0f}\")",
                    ),
                    (
                        "self.lbl_speed_z.setText(f\"{speeds['z']:.1f}\")",
                        "self.lbl_speed_z.setText(f\"{float(speeds.get('z', 0)):.1f}\")",
                    ),
                    (
                        "self.lbl_speed_p.setText(f\"{speeds['p']:.1f}\")",
                        "self.lbl_speed_p.setText(f\"{float(speeds.get('p', 0)):.1f}\")",
                    ),
                ]
                for old, new in replacements_dq:
                    if old in content:
                        content = content.replace(old, new, 1)
                        any_replaced = True

                if any_replaced:
                    print(f"  {GREEN}OK{RESET}: B1 — dashboard speed lines made safe (dq)")
                    _applied += 1
                else:
                    print(f"  {RED}MISS{RESET}: B1 — could not find speed display lines")
                    print(f"         Search for 'lbl_speed_xy.setText' in dashboard.py")
                    _failed += 1

    write(path, content)
    if not ast_check(path):
        print(f"  {RED}FATAL{RESET}: dashboard.py AST broken after patch!")
        _failed += 1


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{BOLD}{'=' * 60}")
    print(" MEBP Speed Display Fix Patch")
    print(f"{'=' * 60}{RESET}\n")

    hint = sys.argv[1] if len(sys.argv) > 1 else None
    root = find_project_root(hint)
    print(f"  Project root: {root}\n")

    print(f"{BOLD}Fix A: StageController.get_speed_info(){RESET}")
    fix_get_speed_info(root)

    print(f"\n{BOLD}Fix B: Dashboard safe speed formatting{RESET}")
    fix_dashboard_speeds(root)

    # Summary
    print(f"\n{BOLD}{'=' * 60}")
    print(f" SUMMARY")
    print(f"{'=' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped}")
    print(f"  {RED}Failed{RESET}:   {_failed}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} fix(es) need manual attention.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All fixes applied successfully!{RESET}")
        print(f"Restart the app to verify: python main.py")


if __name__ == "__main__":
    main()
