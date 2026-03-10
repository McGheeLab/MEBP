#!/usr/bin/env python3
"""
Fix ProScan II detection: firmware_query "COMP" doesn't match response_pattern.

Problem: firmware_query is "COMP" (returns "0") but response_pattern expects
         comma-separated numbers like "^-?\\d+,-?\\d+". Mismatch → detection fails.

Fix: Change firmware_query to "P" (position query). P always works on ProScan II
     and returns "-5228,-5942,0" which matches the pattern perfectly.

Run: python patches/patch_fix_proscan_detection.py
"""

import json
import sys
from pathlib import Path


def find_root() -> Path:
    here = Path(__file__).resolve().parent
    for p in [here, here.parent, here.parent.parent, Path.cwd()]:
        if (p / "SupportClasses").is_dir():
            return p
    print("ERROR: Could not find MEBP project root")
    sys.exit(1)


def main():
    root = find_root()
    json_path = root / "config" / "controllers" / "proscan_ii.json"

    print("=" * 60)
    print("  Fix: ProScan II detection query mismatch")
    print("=" * 60)
    print(f"  File: {json_path}")

    if not json_path.exists():
        print(f"  ERROR: {json_path} not found")
        sys.exit(1)

    with open(json_path, "r") as f:
        config = json.load(f)

    det = config.get("detection", {})
    current_query = det.get("firmware_query", "???")
    current_pattern = det.get("response_pattern", "???")

    print(f"\n  Current firmware_query:   {current_query}")
    print(f"  Current response_pattern: {current_pattern}")

    if current_query == "P":
        print("\n  ✅ Already correct — firmware_query is 'P'")
        return

    # Fix: P returns "x,y,z" which matches ^-?\d+,-?\d+
    det["firmware_query"] = "P"
    det["notes"] = (
        "v7.2.8-fix: Use P (position query) for detection. "
        "V returns E,4 on ProScan II. COMP returns 0/1 which "
        "doesn't match the response_pattern. P always works and "
        "returns comma-separated numbers that match the pattern."
    )
    config["detection"] = det

    with open(json_path, "w") as f:
        json.dump(config, f, indent=4)

    print(f"\n  Fixed firmware_query: 'COMP' → 'P'")
    print(f"  Pattern '{current_pattern}' will now match P response (e.g. '-5228,-5942,0')")
    print("\n  ✅ ProScan II auto-detection should now work")
    print("=" * 60)


if __name__ == "__main__":
    main()
