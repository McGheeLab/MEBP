#!/usr/bin/env python3
"""
fix_settings_page.py — Fix the settings_page.py patch insertion error.

The v7.2 patch inserted set_hardware_config() after the docstring of
set_microsteps_per_micron(), which pushed `self._microsteps_per_micron = value`
into the wrong method. This script fixes that.

Run from MEBP project root:
    python fix_settings_page.py
"""

import os
import sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


def main():
    path = "gui/pages/settings_page.py"
    if not os.path.isfile(path):
        print(f"{RED}ERROR{RESET}: {path} not found. Run from MEBP project root.")
        sys.exit(1)

    with open(path, "r") as f:
        content = f.read()

    # The broken pattern looks like:
    #     def set_microsteps_per_micron(self, value: float):
    #         """Update the conversion factor and refresh related UI."""
    #
    #     _hardware_config = None
    #
    #     def set_hardware_config(self, config):
    #         """v7.2: Receive hardware config for dual mm/µL display."""
    #         self._hardware_config = config
    #
    #         self._microsteps_per_micron = value   <-- WRONG: 'value' not in scope
    #
    # We need to move that line back into set_microsteps_per_micron.

    broken = '''    def set_microsteps_per_micron(self, value: float):
        """Update the conversion factor and refresh related UI."""

    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Receive hardware config for dual mm/µL display."""
        self._hardware_config = config

        self._microsteps_per_micron = value'''

    fixed = '''    def set_microsteps_per_micron(self, value: float):
        """Update the conversion factor and refresh related UI."""
        self._microsteps_per_micron = value

    _hardware_config = None

    def set_hardware_config(self, config):
        """v7.2: Receive hardware config for dual mm/µL display."""
        self._hardware_config = config'''

    if broken in content:
        content = content.replace(broken, fixed, 1)
        with open(path, "w") as f:
            f.write(content)
        print(f"{GREEN}OK{RESET}: Fixed set_hardware_config insertion in settings_page.py")
    elif fixed in content:
        print(f"{GREEN}Already fixed{RESET}: settings_page.py looks correct")
    else:
        # Try a more flexible match
        if ("def set_hardware_config(self, config):" in content and
            "self._microsteps_per_micron = value" in content):
            # Find and check if value line is inside set_hardware_config
            lines = content.split("\n")
            in_set_hardware = False
            fix_line = None
            for i, line in enumerate(lines):
                if "def set_hardware_config(self, config):" in line:
                    in_set_hardware = True
                elif in_set_hardware and line.strip().startswith("def "):
                    in_set_hardware = False
                elif in_set_hardware and "self._microsteps_per_micron = value" in line:
                    fix_line = i
                    break

            if fix_line is not None:
                # Remove the misplaced line
                bad_line = lines.pop(fix_line)
                # Find set_microsteps_per_micron docstring and insert after it
                for i, line in enumerate(lines):
                    if "def set_microsteps_per_micron(self, value: float):" in line:
                        # Find the docstring end
                        for j in range(i + 1, min(i + 5, len(lines))):
                            if '"""' in lines[j] and j > i:
                                lines.insert(j + 1, "        self._microsteps_per_micron = value")
                                break
                        break

                content = "\n".join(lines)
                with open(path, "w") as f:
                    f.write(content)
                print(f"{GREEN}OK{RESET}: Fixed set_hardware_config insertion (flexible match)")
            else:
                print(f"settings_page.py structure not recognized — may need manual fix")
        else:
            print(f"settings_page.py doesn't contain the expected broken pattern — may already be correct")


if __name__ == "__main__":
    main()
