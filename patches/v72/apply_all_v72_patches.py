#!/usr/bin/env python3
"""
apply_all_v72_patches.py — Master script to apply all MEBP v7.2 patches.

Run from the MEBP project root:
    python apply_all_v72_patches.py

Applies patches in order:
    1. StageController + __init__ + Settings   (core µL methods)
    2. PrintManager                             (µL commands + migration)
    3. Dashboard                                (µL pump display)
    4. Print Setup                              (µL settings panel)

Prerequisites (must already be in place):
    - SupportClasses/HardwareConfig.py    (new file)
    - gui/pages/hardware_setup.py         (new file)
    - gui/pages/jog_control.py            (replacement file)
    - gui/app.py                          (replacement file)
    - config/hardware/sample_setup.json   (new file)

These files are delivered separately and should be copied first.
"""

import os
import sys
import subprocess

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"


def run_patch(script_path):
    """Run a patch script and report success/failure."""
    if not os.path.isfile(script_path):
        print(f"  {RED}ERROR{RESET}: {script_path} not found!")
        return False

    script_name = os.path.basename(script_path)
    print(f"\n{'═'*60}")
    print(f"  Running: {script_name}")
    print(f"{'═'*60}")

    result = subprocess.run(
        [sys.executable, script_path],
        capture_output=False,
    )

    if result.returncode == 0:
        print(f"  {GREEN}✓ {script_name} completed successfully{RESET}")
        return True
    else:
        print(f"  {RED}✗ {script_name} failed (exit code {result.returncode}){RESET}")
        return False


def check_prerequisites():
    """Check that new files are in place before patching."""
    required_new_files = [
        "SupportClasses/HardwareConfig.py",
        "gui/pages/hardware_setup.py",
    ]
    required_existing_files = [
        "SupportClasses/StageController.py",
        "SupportClasses/PrintManager.py",
        "SupportClasses/__init__.py",
        "gui/pages/dashboard.py",
        "gui/pages/print_setup.py",
    ]

    all_ok = True
    print(f"\n{BOLD}Checking prerequisites...{RESET}")

    for f in required_new_files:
        if os.path.isfile(f):
            print(f"  {GREEN}✓{RESET} {f}")
        else:
            print(f"  {RED}✗{RESET} {f} — MISSING (copy from deliverables first!)")
            all_ok = False

    for f in required_existing_files:
        if os.path.isfile(f):
            print(f"  {GREEN}✓{RESET} {f}")
        else:
            print(f"  {RED}✗{RESET} {f} — MISSING (is this the right directory?)")
            all_ok = False

    return all_ok


def find_project_root():
    """Find the MEBP project root by searching for SupportClasses/."""
    # Try current directory
    if os.path.isdir("SupportClasses"):
        return os.getcwd()
    # Try parent directories (e.g. running from patches/v72/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        candidate = os.path.normpath(script_dir)
        if os.path.isdir(os.path.join(candidate, "SupportClasses")):
            return candidate
        script_dir = os.path.dirname(script_dir)
    return None


def main():
    print(f"\n{BOLD}{'═'*60}")
    print(f"  MEBP v7.2 — Complete Patch Application")
    print(f"{'═'*60}{RESET}")

    # Find project root
    project_root = find_project_root()
    if project_root is None:
        print(f"\n{RED}ERROR{RESET}: Cannot find MEBP project root (SupportClasses/ not found).")
        print(f"Current directory: {os.getcwd()}")
        print(f"Run from the MEBP project root or from patches/v72/")
        sys.exit(1)

    # Change to project root for all patches
    patch_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(project_root)
    print(f"Project root: {project_root}")
    print(f"Patch dir:    {patch_dir}")

    if not check_prerequisites():
        print(f"\n{YELLOW}Some prerequisites are missing. Continue anyway? (y/n){RESET}")
        resp = input().strip().lower()
        if resp != 'y':
            sys.exit(1)

    # Define patch order — resolve paths relative to patch directory
    patches = [
        (os.path.join(patch_dir, "apply_v72_patches.py"), "Session 1: StageController + __init__ + Settings"),
        (os.path.join(patch_dir, "apply_v72_printmanager_patch.py"), "Session 2: PrintManager µL migration"),
        (os.path.join(patch_dir, "apply_v72_dashboard_patch.py"), "Session 2: Dashboard µL display"),
        (os.path.join(patch_dir, "apply_v72_printsetup_patch.py"), "Session 2: Print Setup µL settings"),
        (os.path.join(patch_dir, "apply_v72_sessions3to6_patch.py"), "Sessions 3-6: Cal + Safety + Xbox + Monitor"),
    ]

    results = []
    for script_path, desc in patches:
        print(f"\n{BOLD}>>> {desc}{RESET}")
        ok = run_patch(script_path)
        results.append((desc, ok))

    # Summary
    print(f"\n{'═'*60}")
    print(f"{BOLD}  PATCH SUMMARY{RESET}")
    print(f"{'═'*60}")
    for desc, ok in results:
        status = f"{GREEN}✓ PASS{RESET}" if ok else f"{RED}✗ FAIL{RESET}"
        print(f"  {status}  {desc}")

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"\n  {passed}/{total} patches applied successfully")

    if passed == total:
        print(f"\n{GREEN}{BOLD}All patches applied! v7.2 upgrade complete.{RESET}")
        print(f"\nNew/replacement files to verify:")
        print(f"  • SupportClasses/HardwareConfig.py  (new)")
        print(f"  • gui/pages/hardware_setup.py       (new)")
        print(f"  • gui/pages/jog_control.py           (replacement)")
        print(f"  • gui/app.py                         (replacement)")
        print(f"  • config/hardware/sample_setup.json  (new)")
        print(f"  • config/prints/sample_v72_print.json (new)")
        print(f"\nRun tests:")
        print(f"  python test_hardware_config.py")
        print(f"  python test_printmanager_v72.py")
    else:
        print(f"\n{YELLOW}Some patches failed. Check output above for details.{RESET}")


if __name__ == "__main__":
    main()
