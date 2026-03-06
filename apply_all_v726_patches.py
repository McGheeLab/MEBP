#!/usr/bin/env python3
"""
MEBP v7.2.6 — Master Patch Runner.

Applies Session 1 and Session 2 patches in dependency order.
Safe to re-run (all patches are idempotent).

Usage:
    python apply_all_v726_patches.py [/path/to/MEBP]
"""

import subprocess
import sys
from pathlib import Path

G = "\033[92m"; R = "\033[91m"; X = "\033[0m"; B = "\033[1m"

def main():
    # Determine project root
    root_arg = sys.argv[1] if len(sys.argv) > 1 else ""
    patches_dir = Path(__file__).resolve().parent

    sessions = [
        ("Session 1: Job Building Pipeline",
         patches_dir / "patch_v726_session1_job_pipeline.py"),
        ("Session 2: Execution Chain Fix",
         patches_dir / "patch_v726_session2_execution_chain.py"),
    ]

    print(f"\n{B}{'=' * 60}")
    print(f"  MEBP v7.2.6 — Master Patch Runner")
    print(f"{'=' * 60}{X}\n")

    all_ok = True
    for name, script in sessions:
        print(f"\n{B}>> {name}{X}")
        if not script.exists():
            print(f"  {R}ERROR: {script.name} not found{X}")
            all_ok = False
            continue

        cmd = [sys.executable, str(script)]
        if root_arg:
            cmd.append(root_arg)

        result = subprocess.run(cmd, capture_output=False)
        if result.returncode != 0:
            print(f"  {R}FAILED with exit code {result.returncode}{X}")
            all_ok = False
            break  # Stop on first failure

    print(f"\n{B}{'=' * 60}{X}")
    if all_ok:
        print(f"  {G}✓ All patches applied successfully.{X}")
        print(f"\n  Next steps:")
        print(f"    1. AST-check: python3 -c \"import ast; ast.parse(open('gui/pages/print_setup.py').read())\"")
        print(f"    2. Import check: python3 -c \"from gui.pages.print_setup import PrintSetupPage\"")
        print(f"    3. Launch: python main.py")
        print(f"    4. Test: HW Setup → Wells → Generate Print → Send to Monitor → Start")
    else:
        print(f"  {R}✗ Some patches failed. Fix issues and re-run.{X}")
        sys.exit(1)


if __name__ == "__main__":
    main()
