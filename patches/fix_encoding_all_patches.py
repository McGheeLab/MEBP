#!/usr/bin/env python3
"""
MEBP v7.2.5 — Fix Windows Console Encoding in All Patch Files

Problem: Windows default console encoding (cp1252) cannot handle Unicode
box-drawing characters (═ ─ ┌ ┐ └ ┘ │ etc.) and µ used in print statements.
This causes UnicodeEncodeError on Windows.

Fix: Injects `sys.stdout.reconfigure(encoding='utf-8', errors='replace')`
at the top of each patch file, right after the existing `import sys` line.
Also injects stderr reconfigure for completeness.

This is idempotent — safe to run multiple times.

Usage:
    python fix_encoding_all_patches.py
    (run from the patches/v725/ directory, or from anywhere)
"""

import os
import sys
from pathlib import Path

PATCH_FILES = [
    "patch_s1_config_load_and_xy_fix.py",
    "patch_s2_pump_ul_and_generate.py",
    "patch_s3_prints_and_hw_sync.py",
    "patch_s3_fix.py",
    "patch_s4_well_redesign.py",
    "patch_s5_colors_and_integration.py",
    "patch_s6_layout_fixes.py",
]

# The encoding fix to inject
ENCODING_FIX = """
# === Windows console encoding fix ===
import sys as _sys
if _sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(_sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')
# === End encoding fix ===
"""

MARKER = "# === Windows console encoding fix ==="


def fix_file(filepath: Path) -> str:
    """Add encoding fix to a single file. Returns status string."""
    if not filepath.exists():
        return "NOT FOUND"

    content = filepath.read_text(encoding="utf-8")

    # Already fixed?
    if MARKER in content:
        return "SKIP (already fixed)"

    # Find the right insertion point: after the module docstring and imports,
    # but before any real code. Best spot: right after `import sys` line.
    # If there's no `import sys`, insert after the first `import` block.

    lines = content.split('\n')
    insert_idx = None

    # Strategy 1: Find `import sys` and insert after it
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped == 'import sys':
            insert_idx = i + 1
            break
        # Also handle `import os, sys` or `import re, sys`
        if stripped.startswith('import ') and 'sys' in stripped.split(','):
            insert_idx = i + 1
            break

    # Strategy 2: If no `import sys` found, find the end of the import block
    if insert_idx is None:
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('import ') or stripped.startswith('from '):
                insert_idx = i + 1  # keep updating to get the LAST import
            elif insert_idx is not None and stripped and not stripped.startswith('#'):
                # We've passed the import block
                break

    if insert_idx is None:
        # Fallback: insert at line 2 (after shebang)
        insert_idx = 1

    # Insert the fix
    lines.insert(insert_idx, ENCODING_FIX)
    filepath.write_text('\n'.join(lines), encoding='utf-8')
    return "FIXED"


def main():
    # Determine patch directory
    script_dir = Path(__file__).resolve().parent

    # Check if we're in the right place
    found = [f for f in PATCH_FILES if (script_dir / f).exists()]
    if not found:
        # Try looking in patches/v725/ relative to CWD
        alt_dir = Path.cwd() / "patches" / "v725"
        if alt_dir.is_dir():
            script_dir = alt_dir
            found = [f for f in PATCH_FILES if (script_dir / f).exists()]

    if not found:
        alt_dir2 = Path.cwd()
        found = [f for f in PATCH_FILES if (alt_dir2 / f).exists()]
        if found:
            script_dir = alt_dir2

    print("MEBP v7.2.5 -- Fix Windows Console Encoding")
    print(f"Patch directory: {script_dir}")
    print(f"Found {len(found)} of {len(PATCH_FILES)} patch files")
    print()

    results = []
    for filename in PATCH_FILES:
        filepath = script_dir / filename
        status = fix_file(filepath)
        results.append((filename, status))
        print(f"  {filename:45s} {status}")

    fixed = sum(1 for _, s in results if s == "FIXED")
    skipped = sum(1 for _, s in results if s.startswith("SKIP"))
    missing = sum(1 for _, s in results if s == "NOT FOUND")

    print(f"\nSummary: {fixed} fixed, {skipped} already done, {missing} not found")

    if missing > 0:
        print(f"\nWARNING: {missing} patch file(s) not found.")
        print("Make sure all patch files are in the same directory.")


if __name__ == "__main__":
    main()
