#!/usr/bin/env python3
"""Delete all .bak* files from the MEBP project."""

import sys, os
from pathlib import Path

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"; B = "\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    print(f"\n{B}=== Clean .bak files ==={X}")
    print(f"  Root: {root}\n")

    bak_files = []
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if ".bak" in f:
                bak_files.append(Path(dirpath) / f)

    if not bak_files:
        print(f"  {G}No .bak files found — already clean.{X}")
        return

    total_mb = sum(f.stat().st_size for f in bak_files) / (1024 * 1024)
    print(f"  Found {len(bak_files)} .bak files ({total_mb:.1f} MB):\n")
    for f in sorted(bak_files):
        size_kb = f.stat().st_size / 1024
        print(f"    {f.relative_to(root)}  ({size_kb:.0f} KB)")

    print()
    confirm = input(f"  Delete all {len(bak_files)} files? [y/N] ").strip().lower()
    if confirm != "y":
        print(f"  {Y}Cancelled.{X}")
        return

    deleted = 0
    for f in bak_files:
        try:
            f.unlink()
            deleted += 1
        except Exception as e:
            print(f"  {R}Failed: {f.name}: {e}{X}")

    print(f"\n  {G}✓ Deleted {deleted}/{len(bak_files)} .bak files ({total_mb:.1f} MB freed){X}")

if __name__ == "__main__":
    main()
