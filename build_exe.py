#!/usr/bin/env python3
"""
build_exe.py — Build MEBP as a standalone executable.

Usage:
    python build_exe.py              # Standard build
    python build_exe.py --clean      # Clean previous builds first
    python build_exe.py --debug      # Build with console + debug info

Output:
    dist/MEBP/MEBP                   # macOS / Linux executable
    dist/MEBP/MEBP.exe               # Windows executable

The dist/MEBP/ folder is fully self-contained and can be distributed
as-is (zip it up, copy to a USB drive, etc.).

Requirements:
    pip install pyinstaller
"""

from __future__ import annotations

import argparse
import platform
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
SPEC_FILE = PROJECT_ROOT / "MEBP.spec"
DIST_DIR = PROJECT_ROOT / "dist"
BUILD_DIR = PROJECT_ROOT / "build"


def check_pyinstaller():
    """Ensure PyInstaller is installed."""
    try:
        import PyInstaller
        print(f"  PyInstaller {PyInstaller.__version__} found")
        return True
    except ImportError:
        return False


def install_pyinstaller():
    """Install PyInstaller via pip."""
    print("Installing PyInstaller...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])
    print("  PyInstaller installed successfully")


def clean_build():
    """Remove previous build artifacts."""
    for d in [BUILD_DIR, DIST_DIR]:
        if d.exists():
            print(f"  Removing {d.relative_to(PROJECT_ROOT)}/")
            shutil.rmtree(d)


def run_build(debug: bool = False):
    """Run PyInstaller with the spec file."""
    cmd = [
        sys.executable, "-m", "PyInstaller",
        str(SPEC_FILE),
        "--noconfirm",          # Overwrite dist/ without asking
    ]

    if debug:
        cmd.append("--log-level=DEBUG")

    print(f"\nRunning: {' '.join(cmd)}\n")
    result = subprocess.run(cmd, cwd=str(PROJECT_ROOT))

    if result.returncode != 0:
        print(f"\nBuild FAILED (exit code {result.returncode})")
        sys.exit(result.returncode)

    return True


def post_build_info():
    """Print info about the build output."""
    app_dir = DIST_DIR / "MEBP"
    if not app_dir.exists():
        print("WARNING: Expected output directory not found!")
        return

    system = platform.system()
    if system == "Windows":
        exe = app_dir / "MEBP.exe"
    else:
        exe = app_dir / "MEBP"

    # Calculate approximate size
    total_size = sum(f.stat().st_size for f in app_dir.rglob("*") if f.is_file())
    size_mb = total_size / (1024 * 1024)

    print("\n" + "=" * 60)
    print("BUILD SUCCESSFUL")
    print("=" * 60)
    print(f"  Output:    {app_dir}")
    print(f"  Executable: {exe}")
    print(f"  Size:      {size_mb:.1f} MB")
    print(f"  Platform:  {system} {platform.machine()}")
    print()
    print("To run:")
    if system == "Windows":
        print(f'  .\\dist\\MEBP\\MEBP.exe')
    else:
        print(f"  ./dist/MEBP/MEBP")
    print()
    print("To distribute:")
    print(f"  Zip or copy the entire dist/MEBP/ folder.")
    print()
    print("Notes:")
    print("  - Settings, print files, and records are stored inside the")
    print("    MEBP folder (in _internal/ alongside config files).")
    print("  - ToupCam camera users: install ToupView or place the SDK")
    print("    (toupcam-master/) next to the executable.")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Build MEBP executable")
    parser.add_argument("--clean", action="store_true",
                        help="Remove build/ and dist/ before building")
    parser.add_argument("--debug", action="store_true",
                        help="Build with console window and debug logging")
    args = parser.parse_args()

    print("=" * 60)
    print("MEBP Executable Builder")
    print("=" * 60)
    print(f"  Project: {PROJECT_ROOT}")
    print(f"  Python:  {sys.version.split()[0]}")
    print(f"  Platform: {platform.system()} {platform.machine()}")
    print()

    # Step 1: Check / install PyInstaller
    if not check_pyinstaller():
        install_pyinstaller()

    # Step 2: Optionally clean
    if args.clean:
        print("\nCleaning previous build...")
        clean_build()

    # Step 3: If debug mode, temporarily patch spec for console=True
    if args.debug:
        print("\nDEBUG MODE: Console window will be visible")
        spec_text = SPEC_FILE.read_text()
        patched = spec_text.replace("console=False", "console=True")
        SPEC_FILE.write_text(patched)
        try:
            run_build(debug=True)
        finally:
            # Restore original spec
            SPEC_FILE.write_text(spec_text)
    else:
        run_build()

    # Step 4: Report results
    post_build_info()


if __name__ == "__main__":
    main()
