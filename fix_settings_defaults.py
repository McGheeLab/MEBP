#!/usr/bin/env python3
"""
fix_settings_defaults.py — Add hardware_config to Settings.py DEFAULTS.

Fixes the one SKIP from apply_v72_patches.py where the anchor text
for Settings.py DEFAULTS didn't match.

Run from MEBP project root:
    python fix_settings_defaults.py
"""

import os
import sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"
BOLD = "\033[1m"


def main():
    # Find Settings.py
    path = "SupportClasses/Settings.py"
    if not os.path.isfile(path):
        print(f"{RED}ERROR{RESET}: {path} not found. Run from MEBP project root.")
        sys.exit(1)

    with open(path, "r") as f:
        content = f.read()

    # Check if already patched
    if '"hardware_config"' in content:
        print(f"{GREEN}Already patched{RESET}: hardware_config already in Settings.py DEFAULTS")
        return

    # Find the fluid_columns section end and insert after it
    anchor = '''    "fluid_columns": {
        # P8.32: Fluid column state persistence (save/restore between sessions)
        # Per-pump fluid column state (oil_uL, buffer_uL, ink_uL, ink_name)
        "P1": None,
        "P2": None,
        "P3": None,
    },
}'''

    insertion = '''    "fluid_columns": {
        # P8.32: Fluid column state persistence (save/restore between sessions)
        # Per-pump fluid column state (oil_uL, buffer_uL, ink_uL, ink_name)
        "P1": None,
        "P2": None,
        "P3": None,
    },
    # v7.2: Hardware configuration persistence
    "hardware_config": {
        "last_config_file": None,           # Path to last loaded hardware config JSON
        "auto_load": True,                  # Auto-load last config on startup
    },
}'''

    if anchor in content:
        content = content.replace(anchor, insertion, 1)
        with open(path, "w") as f:
            f.write(content)
        print(f"{GREEN}OK{RESET}: Added hardware_config section to Settings.py DEFAULTS")
    else:
        # Try a simpler anchor — just the closing brace of DEFAULTS
        # Find the last entry before the closing }
        alt_anchor = '        "P3": None,\n    },\n}'
        if alt_anchor in content:
            alt_insertion = ('        "P3": None,\n    },\n'
                           '    # v7.2: Hardware configuration persistence\n'
                           '    "hardware_config": {\n'
                           '        "last_config_file": None,\n'
                           '        "auto_load": True,\n'
                           '    },\n}')
            content = content.replace(alt_anchor, alt_insertion, 1)
            with open(path, "w") as f:
                f.write(content)
            print(f"{GREEN}OK{RESET}: Added hardware_config section to Settings.py DEFAULTS (alt anchor)")
        else:
            print(f"{RED}FAIL{RESET}: Could not find insertion point in Settings.py.")
            print("Please manually add to the DEFAULTS dict at the end:")
            print('    "hardware_config": {')
            print('        "last_config_file": None,')
            print('        "auto_load": True,')
            print("    },")


if __name__ == "__main__":
    main()
