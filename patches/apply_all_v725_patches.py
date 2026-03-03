#!/usr/bin/env python3
"""
MEBP v7.2.5 — Apply All Patches
Runs Session 1-5 patches in order.

Usage:
    python apply_all_v725_patches.py [/path/to/MEBP]
"""

import subprocess
import sys
from pathlib import Path

BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
RESET  = "\033[0m"

PATCHES = [
    ("Session 1: Config Load + XY Motion Fix",    "patch_s1_config_load_and_xy_fix.py"),
    ("Session 2: Pump µL + Generate Button",       "patch_s2_pump_ul_and_generate.py"),
    ("Session 3: Prints List + HW Sync",           "patch_s3_prints_and_hw_sync.py"),
    ("Session 4: Well Setup Redesign",             "patch_s4_well_redesign.py"),
    ("Session 5: Colors + Integration",            "patch_s5_colors_and_integration.py"),
]


def main():
    # Determine project root
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        # Auto-detect
        for candidate in [Path.cwd(), Path.cwd().parent, Path(__file__).resolve().parent.parent]:
            if (candidate / "gui" / "pages").is_dir():
                root = candidate
                break
        else:
            print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
            print("Usage: python apply_all_v725_patches.py /path/to/MEBP")
            sys.exit(1)

    patches_dir = Path(__file__).resolve().parent

    print(f"\n{BOLD}{'═' * 60}")
    print(f" MEBP v7.2.5 — Master Patch Runner")
    print(f"{'═' * 60}{RESET}")
    print(f"Project root: {root}")
    print(f"Patches dir:  {patches_dir}")
    print()

    results = []
    for name, filename in PATCHES:
        patch_path = patches_dir / filename
        if not patch_path.exists():
            print(f"  {YELLOW}SKIP{RESET}: {name} — {filename} not found")
            results.append((name, "SKIPPED"))
            continue

        print(f"\n{BOLD}▶ {name}{RESET}")
        print(f"  Running: {filename}")

        try:
            result = subprocess.run(
                [sys.executable, str(patch_path), str(root)],
                capture_output=True, text=True, timeout=120
            )
            print(result.stdout)
            if result.stderr:
                print(f"  {YELLOW}stderr{RESET}: {result.stderr[:200]}")

            if result.returncode == 0:
                results.append((name, "OK"))
            else:
                results.append((name, "FAILED"))
                print(f"  {RED}FAILED{RESET}: Exit code {result.returncode}")

        except subprocess.TimeoutExpired:
            results.append((name, "TIMEOUT"))
            print(f"  {RED}TIMEOUT{RESET}")
        except Exception as e:
            results.append((name, f"ERROR: {e}"))
            print(f"  {RED}ERROR{RESET}: {e}")

    # Final summary
    print(f"\n{BOLD}{'═' * 60}")
    print(f" FINAL SUMMARY")
    print(f"{'═' * 60}{RESET}")
    for name, status in results:
        color = GREEN if status == "OK" else YELLOW if status == "SKIPPED" else RED
        print(f"  {color}{status:8s}{RESET}  {name}")

    failed = sum(1 for _, s in results if s not in ("OK", "SKIPPED"))
    if failed > 0:
        print(f"\n{RED}{failed} patch(es) failed!{RESET}")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All patches applied successfully!{RESET}")


if __name__ == "__main__":
    main()
