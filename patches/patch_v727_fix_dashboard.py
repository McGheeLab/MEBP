#!/usr/bin/env python3
"""
MEBP v7.2.7 Fix — Repairs dashboard.py AST error from first patch run.

The original patch's S3-B regex matched `_update_conn_status("xbox", ...))`
up to the FIRST `)` (after "alive"), leaving the outer `)` orphaned.

This script:
  1. Restores dashboard.py from the .bak backup
  2. Re-applies S3-A (_connect_xbox with thread mode) correctly
  3. Re-applies S3-B (Xbox tooltip) with fixed regex
"""

import ast
import glob
import os
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"


def find_root() -> Path:
    candidates = [
        Path.cwd(),
        Path(__file__).resolve().parent,
        Path(__file__).resolve().parent.parent,
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root.{RESET}")
    sys.exit(1)


def find_method(content: str, name: str, indent: int = 4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def main():
    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7 Fix — Dashboard AST Repair")
    print(f"{'='*60}{RESET}\n")

    root = find_root()
    print(f"Project root: {root}\n")

    dash_path = root / "gui" / "pages" / "dashboard.py"

    # ── Step 1: Restore from backup ───────────────────────────────
    # Find the most recent .bak_v727_* file
    bak_pattern = str(dash_path) + ".bak_v727_*"
    bak_files = sorted(glob.glob(bak_pattern))

    if bak_files:
        latest_bak = bak_files[-1]
        print(f"  Restoring from backup: {os.path.basename(latest_bak)}")
        shutil.copy2(latest_bak, dash_path)

        # Verify the restored file is valid
        content = dash_path.read_text(encoding="utf-8")
        try:
            ast.parse(content)
            print(f"  {GREEN}✓ Restored file AST-valid{RESET}")
        except SyntaxError as e:
            print(f"  {RED}✗ Restored file has AST error: {e}{RESET}")
            print(f"  Try restoring manually from an older backup.")
            return 1
    else:
        print(f"  {YELLOW}No .bak_v727 backup found — working with current file{RESET}")
        content = dash_path.read_text(encoding="utf-8")
        try:
            ast.parse(content)
            print(f"  {GREEN}✓ Current file AST-valid{RESET}")
        except SyntaxError as e:
            print(f"  {RED}✗ Current file has AST error: {e}{RESET}")
            # Try any other backup
            all_baks = sorted(glob.glob(str(dash_path) + ".bak*"))
            if all_baks:
                print(f"  Trying oldest backup: {os.path.basename(all_baks[0])}")
                shutil.copy2(all_baks[0], dash_path)
                content = dash_path.read_text(encoding="utf-8")
            else:
                print(f"  No backups available. Cannot proceed.")
                return 1

    original = content

    # ── Step 2: Apply S3-A — _connect_xbox with thread mode ──────
    marker_s3a = "v7.2.7: thread mode option"
    if marker_s3a not in content:
        cx_match = find_method(content, "_connect_xbox")
        if cx_match:
            new_method = (
                '    def _connect_xbox(self):\n'
                f'        """Connect Xbox controller. {marker_s3a}"""\n'
                '        try:\n'
                '            import platform\n'
                '            use_thread = platform.system() == "Darwin"\n'
                '            if use_thread:\n'
                '                logger.info("macOS detected — using thread mode for Bluetooth compatibility")\n'
                '            mapping = getattr(self.controller, "_mapping_file",\n'
                '                              "current_button_mapping.json")\n'
                '            self.controller.connect_xbox(mapping_file=mapping, use_thread=use_thread)\n'
                '        except Exception as e:\n'
                '            logger.error(f"Xbox connect failed: {e}")\n'
                '\n'
            )
            content = content[:cx_match.start()] + new_method + content[cx_match.end():]
            print(f"  {GREEN}✓ S3-A: _connect_xbox enhanced with thread mode{RESET}")
        else:
            print(f"  {RED}✗ S3-A: _connect_xbox method not found{RESET}")
    else:
        print(f"  {YELLOW}○ S3-A: Already applied{RESET}")

    # ── Step 3: Apply S3-B — Xbox tooltip (FIXED regex) ──────────
    marker_s3b = "v7.2.7: xbox tooltip"
    if marker_s3b not in content:
        # FIXED: Match the FULL line including the double )) at end
        # The line is: self._update_conn_status("xbox", _xbox_st in ("connected", "alive"))
        # We need to match through BOTH closing parens
        xbox_block = re.search(
            r'(        _xbox_st\s*=\s*getattr\(self\.controller,\s*"xbox_status",\s*"disconnected"\)\s*\n)'
            r'(        self\._update_conn_status\("xbox",\s*_xbox_st\s+in\s+\("connected",\s*"alive"\)\))',
            content
        )
        if xbox_block:
            old_block = xbox_block.group(0)
            new_block = (
                f'        _xbox_st = getattr(self.controller, "xbox_status", "disconnected")  # {marker_s3b}\n'
                '        self._update_conn_status("xbox", _xbox_st in ("connected", "alive"))\n'
                '        # v7.2.7: Update Xbox tooltip with status detail\n'
                '        _xbox_dot = getattr(self, "ctx_dot_xbox", None)\n'
                '        if _xbox_dot:\n'
                '            if _xbox_st == "waiting":\n'
                '                _xbox_dot.setToolTip("Searching for Xbox controller...")\n'
                '            elif _xbox_st in ("connected", "alive"):\n'
                '                _xbox_dot.setToolTip("Xbox controller active")\n'
                '            elif _xbox_st == "unknown":\n'
                '                _xbox_dot.setToolTip("Xbox worker running, status unknown")\n'
                '            else:\n'
                '                _xbox_dot.setToolTip("Xbox controller disconnected")'
            )
            content = content.replace(old_block, new_block, 1)
            print(f"  {GREEN}✓ S3-B: Xbox tooltip added (fixed regex){RESET}")
        else:
            print(f"  {YELLOW}○ S3-B: Xbox status block not found (may use different format){RESET}")
    else:
        print(f"  {YELLOW}○ S3-B: Already applied{RESET}")

    # ── Step 4: AST verify and write ──────────────────────────────
    if content != original:
        try:
            ast.parse(content)
            print(f"\n  {GREEN}✓ Final AST check passed{RESET}")
        except SyntaxError as e:
            print(f"\n  {RED}✗ Final AST check FAILED: {e}{RESET}")
            print(f"  Dashboard NOT modified. Check the error above.")
            return 1

        # Create fresh backup
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = dash_path.with_suffix(f".bak_v727fix_{ts}")
        shutil.copy2(dash_path, backup)
        dash_path.write_text(content, encoding="utf-8")
        print(f"  {GREEN}→ dashboard.py written successfully{RESET}")
    else:
        print(f"\n  {YELLOW}→ dashboard.py unchanged (all changes already present){RESET}")

    print(f"\n  {GREEN}Done! Dashboard repaired.{RESET}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
