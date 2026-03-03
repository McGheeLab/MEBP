#!/usr/bin/env python3
"""
apply_all_v724_patches.py -- Master runner for MEBP v7.2.4 patches.

Applies Session 1 through Session 5 patches in dependency order.
Each patch is run as a subprocess so failures are isolated.

Usage:
    python apply_all_v724_patches.py /path/to/MEBP
    python apply_all_v724_patches.py              # auto-detect from cwd
"""

import subprocess
import sys
import os
from pathlib import Path


# Ordered list of patches to apply
PATCHES = [
    ("Session 1: Styles + Propagation",        "patch_s1_styles_and_propagation.py"),
    ("Session 2: Jog + File Browser",           "patch_s2_jog_and_filebrowser.py"),
    ("Session 2 Supplement: Controller + Tests", "patch_s2_supplement.py"),
    ("Session 3: Pump-Ink + Needle Channels",   "patch_s3_pump_ink_needle_channels.py"),
    ("Session 4: Preview Overhaul",             "patch_s4_preview_overhaul.py"),
    ("Session 5: Plan + Validation",            "patch_s5_plan_and_validation.py"),
]


def find_project_root(hint=None):
    """Locate MEBP project root."""
    if hint:
        p = Path(hint)
        if p.is_dir() and (p / "gui" / "app.py").exists():
            return p
        raise FileNotFoundError(f"Not a valid MEBP project: {p}")

    # Try cwd and parents
    for candidate in [Path.cwd()] + list(Path.cwd().parents):
        if (candidate / "gui" / "app.py").exists():
            return candidate
    raise FileNotFoundError(
        "Could not find MEBP project root. Pass path as argument.")


def main():
    hint = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        root = find_project_root(hint)
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("MEBP v7.2.4 Patch Runner")
    print(f"Project root: {root}")
    print(f"Patches to apply: {len(PATCHES)}")
    print("=" * 60)

    patch_dir = Path(__file__).parent
    results = []

    for i, (label, filename) in enumerate(PATCHES, 1):
        patch_path = patch_dir / filename
        print(f"\n[{i}/{len(PATCHES)}] {label}")
        print(f"  File: {filename}")

        if not patch_path.exists():
            print(f"  SKIPPED -- patch file not found: {patch_path}")
            results.append((label, "SKIPPED"))
            continue

        try:
            result = subprocess.run(
                [sys.executable, str(patch_path), str(root)],
                capture_output=True,
                text=True,
                timeout=120,
            )
            if result.returncode == 0:
                print("  OK")
                for line in result.stdout.strip().splitlines()[-3:]:
                    print(f"    {line}")
                results.append((label, "OK"))
            else:
                print(f"  FAILED (exit code {result.returncode})")
                if result.stderr:
                    for line in result.stderr.strip().splitlines()[:5]:
                        print(f"    {line}")
                results.append((label, "FAILED"))

        except subprocess.TimeoutExpired:
            print("  TIMEOUT (>120s)")
            results.append((label, "TIMEOUT"))
        except Exception as e:
            print(f"  ERROR: {e}")
            results.append((label, f"ERROR: {e}"))

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    ok_count = sum(1 for _, s in results if s == "OK")
    skip_count = sum(1 for _, s in results if s == "SKIPPED")
    fail_count = len(results) - ok_count - skip_count

    for label, status in results:
        icon = {"OK": "V", "SKIPPED": "-", "FAILED": "X"}.get(status, "X")
        print(f"  [{icon}] {label}: {status}")

    print(f"\n  {ok_count} OK / {skip_count} skipped / {fail_count} failed")

    if fail_count > 0:
        print("\nSome patches failed. Review output above and apply manually.")
        sys.exit(1)
    else:
        print(f"\nAll patches applied successfully to {root}")
        print("Run tests:")
        print("  python -m unittest discover -s tests -p 'test_v724_*.py' -v")


if __name__ == "__main__":
    main()
