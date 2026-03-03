#!/usr/bin/env python3
"""
patch_s3_pump_ink_needle_channels.py — MEBP v7.2.4 Session 3 Patch

Applies Session 3 changes:
  - HardwareConfig: pump_ink_map, ink_pump_map, needle_channel_pump_map
  - hardware_setup.py: reordered UI, exclusive inks, channel mapping

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v724/patch_s3_pump_ink_needle_channels.py

This script copies the Session 3 files into the correct project locations.
It is idempotent (safe to run multiple times).
"""

import os
import sys
import shutil
from pathlib import Path

# ANSI colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


def find_project_root() -> Path:
    """Find the MEBP project root."""
    # Check common locations
    candidates = [
        Path("."),
        Path(__file__).parent.parent.parent,  # patches/v724/ → project root
    ]
    for candidate in candidates:
        if (candidate / "SupportClasses").is_dir() and (candidate / "gui").is_dir():
            return candidate.resolve()
    print(f"{RED}ERROR: Could not find MEBP project root.{RESET}")
    print("Run this script from the project root or ensure")
    print("SupportClasses/ and gui/ directories exist.")
    sys.exit(1)


def copy_file(src: Path, dst: Path, desc: str):
    """Copy a file with logging."""
    if not src.exists():
        print(f"  {RED}SKIP{RESET}: Source not found: {src}")
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    print(f"  {GREEN}OK{RESET}: {desc} → {dst}")
    return True


def main():
    root = find_project_root()
    patch_dir = Path(__file__).parent

    print(f"\n{'='*60}")
    print(f"MEBP v7.2.4 — Session 3 Patch")
    print(f"Pump-Ink Assignment + Needle-Channel Mapping")
    print(f"{'='*60}")
    print(f"Project root: {root}")
    print()

    results = []

    # 1. HardwareConfig.py
    results.append(copy_file(
        patch_dir / "SupportClasses" / "HardwareConfig.py",
        root / "SupportClasses" / "HardwareConfig.py",
        "HardwareConfig.py (pump-ink maps + channel mapping)",
    ))

    # 2. hardware_setup.py
    results.append(copy_file(
        patch_dir  / "gui" / "pages" / "hardware_setup.py",
        root / "gui" / "pages" / "hardware_setup.py",
        "hardware_setup.py (reordered UI + channel mapping section)",
    ))

    # 3. Tests
    results.append(copy_file(
        patch_dir  / "tests" / "test_v724_session3.py",
        root / "tests" / "test_v724_session3.py",
        "test_v724_session3.py (28 test cases)",
    ))

    print(f"\n{'='*60}")
    ok = sum(1 for r in results if r)
    total = len(results)
    if ok == total:
        print(f"{GREEN}SUCCESS: All {total} files applied.{RESET}")
    else:
        print(f"{YELLOW}PARTIAL: {ok}/{total} files applied.{RESET}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
