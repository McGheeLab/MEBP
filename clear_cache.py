#!/usr/bin/env python3
"""Delete all __pycache__ directories from the MEBP project."""

import shutil, sys
from pathlib import Path

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.cwd()
count = 0
for d in root.rglob("__pycache__"):
    if d.is_dir():
        shutil.rmtree(d)
        print(f"  Deleted: {d.relative_to(root)}")
        count += 1
print(f"\n  Cleaned {count} __pycache__ directories")
