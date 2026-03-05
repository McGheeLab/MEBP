#!/usr/bin/env python3
"""
MEBP v7.2.5 — Apply All Patches (Corrected)

Runs Session 1-5 patches in dependency order, including the Session 3 fix
patch that resolves the C2 anchor mismatch and verifies A4/A5 false negatives.

Usage:
    python apply_all_v725_patches.py [/path/to/MEBP]

Patch Chain:
    S1: Config Load + XY Motion Fix
    S2: Pump µL + Generate Print Button
    S3: Prints List + HW Ink/Rosette Sync  (A4/A5 MISSes are expected — false negatives)
    S3fix: Verify A4/A5, fix C2 regex anchor
    S4: Well Setup Redesign + Zoom Controls
    S5: Well Border Colors + Integration Wiring
    S6: Layout Fixes (pump header labels + side-by-side summary)
"""

import subprocess
import sys
import os
from pathlib import Path

# Windows console encoding fix
if sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')
    os.environ["PYTHONIOENCODING"] = "utf-8"

BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
RESET  = "\033[0m"

# Patch definitions: (description, filename, allow_fail)
# allow_fail=True means a non-zero exit is tolerated (e.g. S3 has known false-negative MISSes)
PATCHES = [
    ("Step 0: Fix Windows encoding in all patch files",
     "fix_encoding_all_patches.py", False),

    ("Session 1: Config Load + XY Motion Fix",
     "patch_s1_config_load_and_xy_fix.py", False),

    ("Session 2: Pump µL + Generate Print Button",
     "patch_s2_pump_ul_and_generate.py", False),

    ("Session 3: Prints List + HW Ink/Rosette Sync",
     "patch_s3_prints_and_hw_sync.py", True),  # A4/A5 false negatives → allow_fail

    ("Session 3 Fix: Verify A4/A5 + Fix C2 Regex",
     "patch_s3_fix.py", False),

    ("Session 4: Well Setup Redesign + Zoom",
     "patch_s4_well_redesign.py", False),

    ("Session 5: Well Border Colors + Integration",
     "patch_s5_colors_and_integration.py", False),

    ("Session 6: Layout Fixes (headers + side-by-side summary)",
     "patch_s6_layout_fixes.py", False),
]


def find_project_root() -> Path:
    """Auto-detect the MEBP project root."""
    candidates = [
        Path.cwd(),
        Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    return None


def run_patch(script_path: str, project_root: str, allow_fail: bool = False) -> tuple[bool, str]:
    """
    Run a patch script as a subprocess.

    Returns (success, detail_msg).
    """
    if not os.path.isfile(script_path):
        return (False, f"File not found: {script_path}")

    # Force UTF-8 output encoding for Windows compatibility
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONLEGACYWINDOWSSTDIO"] = "0"

    try:
        result = subprocess.run(
            [sys.executable, script_path, project_root],
            capture_output=True,
            text=True,
            timeout=120,
            env=env,
            encoding="utf-8",
            errors="replace",
        )

        # Print output live
        if result.stdout:
            print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="")

        if result.returncode == 0:
            return (True, "OK")
        elif allow_fail:
            return (True, f"Exit {result.returncode} (allowed — known false negatives)")
        else:
            return (False, f"Exit code {result.returncode}")

    except subprocess.TimeoutExpired:
        return (False, "Timeout (120s)")
    except Exception as e:
        return (False, str(e))


def main():
    # Determine project root
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
        if not (root / "gui" / "pages").is_dir():
            print(f"{RED}ERROR{RESET}: {root} does not appear to be an MEBP project.")
            sys.exit(1)
    else:
        root = find_project_root()
        if root is None:
            print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
            print("Pass the project path as argument:")
            print("    python apply_all_v725_patches.py /path/to/MEBP")
            sys.exit(1)

    # Determine patch directory (same dir as this script)
    patch_dir = Path(__file__).resolve().parent

    print(f"\n{BOLD}{'═' * 60}")
    print(f"  MEBP v7.2.5 — Apply All Patches (Corrected)")
    print(f"{'═' * 60}{RESET}")
    print(f"  Project root: {root}")
    print(f"  Patch dir:    {patch_dir}")
    print(f"  Patches:      {len(PATCHES)}")

    # Verify all patch files exist before starting
    print(f"\n{BOLD}Checking patch files...{RESET}")
    missing = []
    for desc, filename, _ in PATCHES:
        script_path = patch_dir / filename
        if script_path.exists():
            print(f"  {GREEN}✓{RESET} {filename}")
        else:
            print(f"  {RED}✗{RESET} {filename} — MISSING")
            missing.append(filename)

    if missing:
        print(f"\n{RED}ERROR{RESET}: {len(missing)} patch file(s) missing!")
        print("Ensure all patch files are in the same directory as this script.")
        sys.exit(1)

    # Apply patches in order
    results = []
    for desc, filename, allow_fail in PATCHES:
        script_path = str(patch_dir / filename)

        print(f"\n{'─' * 60}")
        print(f"{BOLD}>>> {desc}{RESET}")
        print(f"    Script: {filename}")
        if allow_fail:
            print(f"    {YELLOW}Note: Known false negatives — non-zero exit tolerated{RESET}")
        print(f"{'─' * 60}")

        ok, detail = run_patch(script_path, str(root), allow_fail)
        results.append((desc, filename, ok, detail))

        if not ok:
            print(f"\n{RED}FAILED{RESET}: {desc}")
            print(f"  Detail: {detail}")
            print(f"\n{YELLOW}Stopping patch chain. Fix the failure above and re-run.{RESET}")
            break

    # Summary
    print(f"\n{BOLD}{'═' * 60}")
    print(f"  PATCH SUMMARY")
    print(f"{'═' * 60}{RESET}")

    passed = 0
    failed = 0
    for desc, filename, ok, detail in results:
        if ok:
            status = f"{GREEN}✓ PASS{RESET}"
            passed += 1
        else:
            status = f"{RED}✗ FAIL{RESET}"
            failed += 1
        extra = f"  ({detail})" if detail != "OK" else ""
        print(f"  {status}  {desc}{extra}")

    total = len(results)
    not_run = len(PATCHES) - total
    if not_run > 0:
        print(f"\n  {YELLOW}⚠ {not_run} patch(es) not run due to earlier failure{RESET}")

    print(f"\n  Passed: {passed}/{len(PATCHES)}")
    print(f"  Failed: {failed}")

    if failed == 0 and passed == len(PATCHES):
        print(f"\n{GREEN}{BOLD}All v7.2.5 patches applied successfully!{RESET}")
        print(f"\nFiles modified:")
        print(f"  gui/pages/hardware_setup.py     — Config load pump fix")
        print(f"  gui/pages/jog_control.py         — XY microns + pump µL + headers")
        print(f"  SupportClasses/StageController.py — move_xy_relative_um()")
        print(f"  gui/app.py                       — Micron-direct note")
        print(f"  gui/pages/print_setup.py         — Generate Print button + prints wiring")
        print(f"  gui/pages/print_objects.py       — Send to Available Prints button")
        print(f"  gui/pages/print_well_setup.py    — Well redesign + colors + splitter layout")
        print(f"  gui/widgets/well_plate_view.py   — Zoom buttons + status borders")
        sys.exit(0)
    else:
        print(f"\n{RED}Some patches failed. Review output above.{RESET}")
        sys.exit(1)


if __name__ == "__main__":
    main()
