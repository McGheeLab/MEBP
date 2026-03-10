#!/usr/bin/env python3
"""
Fix double-escape bug in XYStage.py _read_response_cr().

Bug: Line 65 compares ch against b"\\r" (2-byte backslash+r) instead of
     b"\r" (1-byte CR character 0x0D). The comparison NEVER matches a
     single byte from spo.read(1), so the function blocks until timeout
     (~500ms) on every call. XY position queries run at 2 Hz not 83 Hz.

Fix: Replace the double-escaped byte literals with proper single-escaped ones.

Run: python patches/patch_fix_cr_escape.py
"""

import ast
import sys
from pathlib import Path


def find_root() -> Path:
    """Find MEBP project root — works from any location."""
    here = Path(__file__).resolve().parent
    for p in [here, here.parent, here.parent.parent, Path.cwd()]:
        if (p / "SupportClasses").is_dir():
            return p
    print("ERROR: Could not find MEBP project root (no SupportClasses/ found)")
    print(f"  Script dir: {here}")
    print(f"  CWD: {Path.cwd()}")
    sys.exit(1)


def main():
    root = find_root()
    xy_path = root / "SupportClasses" / "XYStage.py"

    print("=" * 60)
    print("  Fix: _read_response_cr double-escape bug")
    print("=" * 60)
    print(f"  File: {xy_path}")

    if not xy_path.exists():
        print(f"  ERROR: {xy_path} not found")
        sys.exit(1)

    # Read as raw bytes so we can see exactly what's in the file
    raw = xy_path.read_bytes()

    # The buggy sequence in the file is literally these bytes:
    #   b"\\r"  →  b  "  \  \  r  "   (6 chars)
    # We want:
    #   b"\r"   →  b  "  \  r  "      (5 chars)
    #
    # Using bytes to match exactly:
    buggy_cr = b'b"\\\\r"'    # file contains: b"\\r"  (backslash backslash r)
    fixed_cr = b'b"\\r"'      # we want:       b"\r"   (backslash r = CR escape)

    buggy_lf = b'b"\\\\n"'    # file contains: b"\\n"
    fixed_lf = b'b"\\n"'      # we want:       b"\n"

    cr_count = raw.count(buggy_cr)
    lf_count = raw.count(buggy_lf)

    print(f"\n  Scanning for double-escaped byte literals...")
    print(f"  Found b\"\\\\r\" (buggy CR): {cr_count} occurrence(s)")
    print(f"  Found b\"\\\\n\" (buggy LF): {lf_count} occurrence(s)")

    if cr_count == 0 and lf_count == 0:
        # Check if already fixed
        if b'b"\\r"' in raw and b'b"\\n"' in raw:
            print("\n  Already fixed — correct single-escape found.")
        else:
            print("\n  WARNING: Neither buggy nor fixed pattern found.")
            print("  Check _read_response_cr manually.")
        return

    # Apply fixes
    fixed = raw.replace(buggy_cr, fixed_cr)
    fixed = fixed.replace(buggy_lf, fixed_lf)

    # Verify the fix
    remaining_cr = fixed.count(buggy_cr)
    remaining_lf = fixed.count(buggy_lf)
    if remaining_cr > 0 or remaining_lf > 0:
        print(f"  ERROR: Still {remaining_cr} buggy CR and {remaining_lf} buggy LF after fix")
        sys.exit(1)

    # AST verify
    try:
        ast.parse(fixed.decode("utf-8", errors="replace"))
    except SyntaxError as e:
        print(f"  ERROR: Fix produced invalid Python: {e}")
        sys.exit(1)

    # Write
    xy_path.write_bytes(fixed)

    applied_cr = cr_count - remaining_cr
    applied_lf = lf_count - remaining_lf
    print(f"\n  Fixed {applied_cr} CR escape(s) and {applied_lf} LF escape(s)")
    print("  AST verification passed")
    print("\n  XY position queries should now complete in ~12ms instead of ~500ms")
    print("  (83 Hz instead of 2 Hz)")
    print("=" * 60)


if __name__ == "__main__":
    main()