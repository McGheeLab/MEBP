#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 3 Fix Patch
Fixes the 3 MISS items from the original Session 3 patch.

A4/A5: _on_new_print and _on_delete_print already call _emit_prints_changed()
        — these were false failures (patch looked for _new_print/_delete_print
          but actual methods are _on_new_print/_on_delete_print). NO FIX NEEDED.

C2:  Connect prints_changed signal — the anchor was a single-line string but
     the actual code uses multi-line hasattr guard with different formatting.

Usage:
    python patch_s3_fix.py [/path/to/MEBP]
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

BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
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
    backup = filepath.with_suffix(f".bak_v725s3fix_{ts}")
    shutil.copy2(filepath, backup)


def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'=' * 60}")
    print(f" MEBP v7.2.5 — Session 3 Fix Patch")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    # ══════════════════════════════════════════════════════════════
    # A4/A5 verification: confirm _on_new_print and _on_delete_print
    #                     already call _emit_prints_changed
    # ══════════════════════════════════════════════════════════════
    po_path = root / "gui" / "pages" / "print_objects.py"
    print(f"\n{'═' * 60}")
    print(f"Verifying: {po_path.name}")
    print(f"{'═' * 60}")

    if po_path.exists():
        po_content = po_path.read_text(encoding="utf-8")

        # Check _on_new_print
        if '_on_new_print' in po_content:
            new_print_section = po_content[po_content.find('def _on_new_print'):
                                            po_content.find('\n    def ', po_content.find('def _on_new_print') + 20)]
            if '_emit_prints_changed' in new_print_section:
                print(f"  {GREEN}VERIFIED{RESET}: A4: _on_new_print already calls _emit_prints_changed()")
            else:
                print(f"  {RED}ISSUE{RESET}: A4: _on_new_print does NOT call _emit_prints_changed()")
                _failed += 1
        else:
            print(f"  {YELLOW}NOTE{RESET}: A4: _on_new_print not found")

        # Check _on_delete_print
        if '_on_delete_print' in po_content:
            delete_section = po_content[po_content.find('def _on_delete_print'):
                                         po_content.find('\n    def ', po_content.find('def _on_delete_print') + 20)]
            if '_emit_prints_changed' in delete_section:
                print(f"  {GREEN}VERIFIED{RESET}: A5: _on_delete_print already calls _emit_prints_changed()")
            else:
                print(f"  {RED}ISSUE{RESET}: A5: _on_delete_print does NOT call _emit_prints_changed()")
                _failed += 1
        else:
            print(f"  {YELLOW}NOTE{RESET}: A5: _on_delete_print not found")

    # ══════════════════════════════════════════════════════════════
    # C2 FIX: Connect prints_changed signal in print_setup.py
    # ══════════════════════════════════════════════════════════════
    ps_path = root / "gui" / "pages" / "print_setup.py"
    print(f"\n{'═' * 60}")
    print(f"Fix C2: {ps_path.name}")
    print(f"{'═' * 60}")

    if not ps_path.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        _failed += 1
    else:
        backup_file(ps_path)
        content = ps_path.read_text(encoding="utf-8")

        # Check if prints_changed is already connected
        if 'prints_changed' in content and 'tab_objects.prints_changed' in content:
            print(f"  {YELLOW}SKIP{RESET}: C2: prints_changed already connected")
            _skipped += 1
        else:
            # The actual anchor is the multi-line hasattr block:
            #     if hasattr(self.tab_objects, 'collections_changed'):
            #         self.tab_objects.collections_changed.connect(
            #             self._on_collections_changed)
            #
            # We need to add prints_changed connection AFTER this block.

            # Use regex to find the collections_changed connection block
            pattern = re.compile(
                r"(        # Objects → Wells \(available print collections\)\n"
                r"        if hasattr\(self\.tab_objects, 'collections_changed'\):\n"
                r"            self\.tab_objects\.collections_changed\.connect\(\n"
                r"                self\._on_collections_changed\))",
                re.MULTILINE
            )

            match = pattern.search(content)
            if match:
                replacement = (
                    match.group(0) + "\n\n"
                    "        # v7.2.5: Also connect prints_changed (primary signal)\n"
                    "        if hasattr(self.tab_objects, 'prints_changed'):\n"
                    "            self.tab_objects.prints_changed.connect(\n"
                    "                self._on_collections_changed)"
                )
                content = content[:match.start()] + replacement + content[match.end():]
                ps_path.write_text(content, encoding="utf-8")
                print(f"  {GREEN}OK{RESET}:   C2: Connected prints_changed signal")
                _applied += 1
            else:
                # Try looser pattern — maybe the comment was already modified by C1
                pattern2 = re.compile(
                    r"(        if hasattr\(self\.tab_objects, 'collections_changed'\):\n"
                    r"            self\.tab_objects\.collections_changed\.connect\(\n"
                    r"                self\._on_collections_changed\))",
                    re.MULTILINE
                )
                match2 = pattern2.search(content)
                if match2:
                    replacement2 = (
                        match2.group(0) + "\n\n"
                        "        # v7.2.5: Also connect prints_changed (primary signal)\n"
                        "        if hasattr(self.tab_objects, 'prints_changed'):\n"
                        "            self.tab_objects.prints_changed.connect(\n"
                        "                self._on_collections_changed)"
                    )
                    content = content[:match2.start()] + replacement2 + content[match2.end():]
                    ps_path.write_text(content, encoding="utf-8")
                    print(f"  {GREEN}OK{RESET}:   C2: Connected prints_changed signal (alt anchor)")
                    _applied += 1
                else:
                    print(f"  {RED}MISS{RESET}: C2: Could not find collections_changed connection block")
                    _failed += 1

    # ══════════════════════════════════════════════════════════════
    # Summary
    # ══════════════════════════════════════════════════════════════
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'═' * 60}")
    print(f" SUMMARY")
    print(f"{'═' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped}")
    print(f"  {RED}Failed{RESET}:   {_failed}")
    print(f"  Total:    {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} issue(s) remain!")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All fixes applied successfully!{RESET}")


if __name__ == "__main__":
    main()
