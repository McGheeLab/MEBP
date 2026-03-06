#!/usr/bin/env python3
"""
MEBP v7.2.6 — Master Deployment Runner

Runs all batch deployments in order.

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v726/deploy_v726_all.py
"""

import sys
import subprocess
from pathlib import Path

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; B = "\033[1m"; X = "\033[0m"
DIR = Path(__file__).parent

BATCHES = [
    ("Batch 1: styles, console, camera, app, jog", "deploy_v726_batch1.py"),
    ("Batch 2: hardware_setup, calibration, print_setup", "deploy_v726_batch2.py"),
    ("Batch 3: print_objects, print_well_setup", "deploy_v726_batch3.py"),
]

def main():
    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}MEBP v7.2.6 — Master Deployment{X}")
    print(f"{B}{'═' * 60}{X}")

    results = []
    for name, script in BATCHES:
        path = DIR / script
        if not path.exists():
            print(f"\n{Y}⊘ {name}: {script} not found — skipping{X}")
            results.append((name, "MISSING"))
            continue

        print(f"\n{B}▶ {name}{X}")
        r = subprocess.run([sys.executable, str(path)], cwd=str(DIR.parent.parent))
        results.append((name, "OK" if r.returncode == 0 else "FAIL"))

    print(f"\n{B}{'═' * 60}{X}")
    for name, status in results:
        c = G if status == "OK" else (Y if status == "MISSING" else R)
        print(f"  {c}{status:>7}{X}  {name}")
    print(f"{B}{'═' * 60}{X}")

    if any(s == "FAIL" for _, s in results):
        sys.exit(1)

if __name__ == "__main__":
    main()
